#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_CONTROLLER_SPEED_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_CONTROLLER_SPEED_H

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace marine_nav_behavior_tree
{

// Pushes a per-task speed value to a controller plugin's default_speed
// parameter via the parameter-service of the target node. Intended to
// run inside a task subtree just before a FollowPath action, so the
// controller's desired_speed_ is set from the task's `speed` field
// (already plumbed into the blackboard as {target_speed} by
// UpdateCurrentTaskData / GetTaskDataDouble).
//
// Skips silently if the requested speed is <= 0 (so the BT can call it
// unconditionally even on tasks that don't carry a speed field).
class SetControllerSpeed : public BT::SyncActionNode
{
public:
  SetControllerSpeed(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::AsyncParametersClient::SharedPtr params_client_;
  std::string cached_target_node_;

  // Dedup across BT ticks. SyncActionNode is re-ticked at the BT loop
  // rate (~100 Hz) once it has returned SUCCESS; without dedup we'd
  // issue a SetParameters call every tick, accumulating pending
  // requests in rmw and flooding the controller's param-change log.
  // Sentinel -1.0 ensures the first valid speed always fires.
  double last_pushed_speed_ = -1.0;
};

} // namespace marine_nav_behavior_tree

#endif
