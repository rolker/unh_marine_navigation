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
      "and skip the update."),
    BT::InputPort<std::string>(
      "target_node",
      "controller_server",
      "Name of the controller server hosting the FollowPath plugin. "
      "Relative names are resolved against the BT node's namespace "
      "(so the default works on any boat namespace). Absolute names "
      "(starting with '/') are used as-is."),
    BT::InputPort<std::string>(
      "parameter_name",
      "FollowPath.default_speed",
      "Parameter on the target node to update with the speed value."),
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

  auto parameter_name = getInput<std::string>("parameter_name");
  if (!parameter_name) {
    throw BT::RuntimeError(name(), " missing [parameter_name]: ", parameter_name.error());
  }

  auto blackboard = config().blackboard;
  auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Resolve target_node against the BT node's namespace if it's a relative
  // name. Keeps `run_tasks.xml` boat-agnostic — the same XML works on bizzy,
  // izzy, or any unnamespaced deployment without per-boat overrides.
  const std::string resolved_target =
    resolveTargetNode(target_node.value(), node->get_namespace());

  if (!params_client_ || cached_target_node_ != resolved_target) {
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, resolved_target);
    cached_target_node_ = resolved_target;
  }

  if (!params_client_->service_is_ready()) {
    // Reset the dedup sentinel so the next tick after the controller
    // recovers re-sends the per-task speed. Without this, a controller_server
    // restart would silently drop the per-task speed: the controller comes
    // back up at its YAML default_speed, but our dedup still has the
    // pre-restart value cached, so the BT skips re-sending.
    // Safe to write here on the BT thread — the completion callback runs on
    // the rclcpp executor thread but only reads/writes last_pushed_speed_
    // via this same tick() path (which holds the BT-thread reentrancy
    // guarantee for SyncActionNode).
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
    rclcpp::Parameter(parameter_name.value(), speed.value())
  };

  // Completion callback consumes the future (so rclcpp prunes the
  // pending request) and surfaces a WARN on a failed SetParameters
  // response — silent failure would let the boat run on whatever
  // default the controller has, which is safety-relevant.
  const std::string target_name_for_log = resolved_target;
  const std::string param_name_for_log = parameter_name.value();
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
            "SetControllerSpeed: %s.%s := %.3f rejected by %s (reason: '%s')",
            target_name_for_log.c_str(), param_name_for_log.c_str(),
            speed_for_log, target_name_for_log.c_str(),
            failure_reason.c_str());
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
    "SetControllerSpeed: requested %s.%s = %.3f",
    resolved_target.c_str(), parameter_name.value().c_str(),
    speed.value());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace marine_nav_behavior_tree
