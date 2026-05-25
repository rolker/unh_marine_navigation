#include "marine_nav_behavior_tree/plugins/action/set_controller_speed.h"

#include <cmath>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace marine_nav_behavior_tree
{

SetControllerSpeed::SetControllerSpeed(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

std::string SetControllerSpeed::resolveTargetNode(
  const std::string & raw_target, const std::string & ns)
{
  if (!raw_target.empty() && raw_target.front() == '/') {
    return raw_target;
  }
  if (ns.empty() || ns == "/") {
    return "/" + raw_target;
  }
  return ns + "/" + raw_target;
}

BT::PortsList SetControllerSpeed::providedPorts()
{
  return {
    BT::InputPort<double>(
      "speed",
      "Target speed (m/s). Values <= 0 are treated as 'no speed in task' "
      "and skip the update. Non-finite values (NaN/Inf) are also skipped "
      "with a throttled WARN."),
    BT::InputPort<std::string>(
      "target_node",
      "controller_server",
      "Name of the controller server hosting the FollowPath plugin. "
      "Relative names are resolved against the BT node's namespace "
      "(so the default works on any boat namespace). Absolute names "
      "(starting with '/') are used as-is."),
    BT::InputPort<std::string>(
      "controller_name",
      "FollowPath",
      "Name of the controller plugin whose `default_speed` is updated. "
      "Pair with the BT's `selected_controller` blackboard variable "
      "(e.g., `controller_name=\"{selected_controller}\"`) so the speed "
      "update follows whichever controller `FollowPath` is using, not "
      "a hardcoded one. The full parameter name set on `target_node` "
      "is `<controller_name>.<parameter_suffix>`."),
    BT::InputPort<std::string>(
      "parameter_suffix",
      "default_speed",
      "Parameter suffix appended to `controller_name` to form the full "
      "parameter path. Override only if a controller plugin exposes its "
      "speed under a non-default name."),
  };
}

BT::NodeStatus SetControllerSpeed::tick()
{
  auto speed = getInput<double>("speed");
  if (!speed) {
    throw BT::RuntimeError(name(), " missing required input [speed]: ", speed.error());
  }

  // Non-finite (NaN/Inf) or non-positive: skip. `(NaN <= 0.0)` is false in
  // IEEE 754, so without an explicit isfinite check NaN would pass the guard,
  // be pushed through SetParameters, and poison desired_speed_ — producing
  // NaN cmd_vel on an autonomous boat.
  // Tasks without a `speed` field land here with speed == default 0.0 (see
  // GetTaskDataDouble usage in run_tasks.xml), which the second branch handles
  // as "no update".
  const double speed_value = speed.value();
  if (!std::isfinite(speed_value)) {
    // Bound the log: a stuck non-finite blackboard would otherwise WARN at BT
    // tick rate. 5 s matches the service-not-ready throttle below.
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "SetControllerSpeed: non-finite speed input (%f); skipping update",
      speed_value);
    return BT::NodeStatus::SUCCESS;
  }
  if (speed_value <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  auto target_node = getInput<std::string>("target_node");
  if (!target_node) {
    throw BT::RuntimeError(name(), " missing [target_node]: ", target_node.error());
  }
  if (target_node.value().empty()) {
    throw BT::RuntimeError(
      name(), " [target_node] is empty — relative resolution against the BT "
      "node's namespace would yield an invalid name (e.g. '/<ns>/'). Set a "
      "non-empty name in the XML or omit the port to use the default.");
  }

  auto controller_name = getInput<std::string>("controller_name");
  if (!controller_name) {
    throw BT::RuntimeError(
      name(), " missing [controller_name]: ", controller_name.error());
  }
  if (controller_name.value().empty()) {
    throw BT::RuntimeError(
      name(), " [controller_name] is empty — the full parameter name would "
      "be \".<parameter_suffix>\" which the param service will reject. "
      "Pair with the BT's selected_controller blackboard variable (e.g. "
      "controller_name=\"{selected_controller}\") or set a non-empty default.");
  }

  auto parameter_suffix = getInput<std::string>("parameter_suffix");
  if (!parameter_suffix) {
    throw BT::RuntimeError(
      name(), " missing [parameter_suffix]: ", parameter_suffix.error());
  }
  if (parameter_suffix.value().empty()) {
    throw BT::RuntimeError(
      name(), " [parameter_suffix] is empty — the full parameter name "
      "would be \"<controller_name>.\" which the param service will reject.");
  }

  // Compose the full parameter path. The plugin sets
  // `<controller_name>.<parameter_suffix>` on `target_node` — e.g. with
  // defaults: "FollowPath.default_speed".
  const std::string parameter_name =
    controller_name.value() + "." + parameter_suffix.value();

  auto blackboard = config().blackboard;
  auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Resolve target_node against the BT node's namespace if it's a relative
  // name. Keeps `run_tasks.xml` boat-agnostic — the same XML works on bizzy,
  // izzy, or any unnamespaced deployment without per-boat overrides.
  const std::string resolved_target =
    resolveTargetNode(target_node.value(), node->get_namespace());

  // Cache invalidation: rebuild the params_client when the resolved
  // target node changes; reset the dedup sentinel when EITHER the
  // target node or the parameter name changes. Without the param-name
  // case, R6's plumbing routes `controller_name="{selected_controller}"`
  // correctly to the SetParameters call, but the dedup state still
  // refers to the previous controller — so if ControllerSelector
  // switches mid-mission and the requested speed happens to match
  // last_pushed_speed_, the new controller silently keeps its old
  // default_speed.
  const bool target_changed =
    !params_client_ || cached_target_node_ != resolved_target;
  const bool param_changed = cached_parameter_name_ != parameter_name;
  if (target_changed) {
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, resolved_target);
    cached_target_node_ = resolved_target;
  }
  if (param_changed) {
    cached_parameter_name_ = parameter_name;
  }
  if (target_changed || param_changed) {
    last_pushed_speed_ = -1.0;
  }

  if (!params_client_->service_is_ready()) {
    // Reset the dedup sentinel so the next tick after the controller
    // recovers re-sends the per-task speed. Without this, a controller_server
    // restart would silently drop the per-task speed: the controller comes
    // back up at its YAML default_speed, but our dedup still has the
    // pre-restart value cached, so the BT skips re-sending.
    // `last_pushed_speed_` is only ever read/written here in tick(), which
    // runs single-threaded on the BT loop — no cross-thread synchronization
    // needed (the completion callback below only inspects results and logs).
    last_pushed_speed_ = -1.0;
    // Throttle: BT re-ticks this node at ~5+ Hz inside ReactiveSequence /
    // PipelineSequence; without throttling a misconfigured target_node
    // (or a controller mid-restart) floods the log. 5 s gives the user
    // enough visibility to notice but not enough to drown the log.
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "SetControllerSpeed: parameter service on %s not ready; skipping speed update",
      resolved_target.c_str());
    // Don't fail the tree just because the controller is mid-restart — the
    // controller will fall back to whatever default_speed it was configured
    // with, which is acceptable.
    return BT::NodeStatus::SUCCESS;
  }

  // Dedup: only fire when the requested speed actually changes. Without
  // this, BT re-ticks at ~100 Hz drown rmw in pending SetParameters
  // requests and the controller's param-change callback in matching
  // log lines. The dedup also bounds the failure-log noise from the
  // completion callback below.
  if (speed.value() == last_pushed_speed_) {
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter(parameter_name, speed.value())
  };

  // Completion callback consumes the future (so rclcpp prunes the
  // pending request) and surfaces a WARN on a failed SetParameters
  // response — silent failure would let the boat run on whatever
  // default the controller has, which is safety-relevant.
  const std::string target_name_for_log = resolved_target;
  const std::string param_name_for_log = parameter_name;
  const double speed_for_log = speed.value();
  auto logger = node->get_logger();
  auto on_complete =
    [logger, target_name_for_log, param_name_for_log, speed_for_log](
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      try {
        auto results = future.get();
        bool any_failed = false;
        std::string failure_reason;
        for (const auto & r : results) {
          if (!r.successful) {
            any_failed = true;
            failure_reason = r.reason;
            break;
          }
        }
        if (any_failed) {
          RCLCPP_WARN(
            logger,
            "SetControllerSpeed: on %s set %s = %.3f rejected (reason: '%s')",
            target_name_for_log.c_str(), param_name_for_log.c_str(),
            speed_for_log, failure_reason.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          logger,
          "SetControllerSpeed: SetParameters future failed on %s: %s",
          target_name_for_log.c_str(), e.what());
      }
    };

  params_client_->set_parameters(params, on_complete);
  last_pushed_speed_ = speed.value();

  RCLCPP_DEBUG(
    node->get_logger(),
    "SetControllerSpeed: on %s set %s = %.3f",
    resolved_target.c_str(), parameter_name.c_str(),
    speed.value());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace marine_nav_behavior_tree
