#include "marine_nav_behavior_tree/plugins/action/set_controller_speed.h"

namespace marine_nav_behavior_tree
{

SetControllerSpeed::SetControllerSpeed(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
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
      "/controller_server",
      "Fully-qualified name of the controller server hosting the FollowPath plugin."),
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

  // Tasks without a `speed` field land here with speed == default 0.0 (see
  // GetTaskDataDouble usage in run_tasks.xml). Skip the update so the
  // controller's existing default_speed stays in effect.
  if (speed.value() <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  auto target_node = getInput<std::string>("target_node");
  if (!target_node) {
    throw BT::RuntimeError(name(), " missing [target_node]: ", target_node.error());
  }

  auto parameter_name = getInput<std::string>("parameter_name");
  if (!parameter_name) {
    throw BT::RuntimeError(name(), " missing [parameter_name]: ", parameter_name.error());
  }

  auto blackboard = config().blackboard;
  auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (!params_client_ || cached_target_node_ != target_node.value()) {
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, target_node.value());
    cached_target_node_ = target_node.value();
  }

  if (!params_client_->service_is_ready()) {
    RCLCPP_WARN(
      node->get_logger(),
      "SetControllerSpeed: parameter service on %s not ready; skipping speed update",
      target_node.value().c_str());
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
  const std::string target_name_for_log = target_node.value();
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
    "SetControllerSpeed: requested %s.%s = %.3f on %s",
    target_node.value().c_str(), parameter_name.value().c_str(),
    speed.value(), target_node.value().c_str());

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree
