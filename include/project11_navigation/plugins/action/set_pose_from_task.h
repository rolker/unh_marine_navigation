#ifndef PROJECT11_NAVIGATION_ACTIONS_SET_POSE_FROM_TASK_H
#define PROJECT11_NAVIGATION_ACTIONS_SET_POSE_FROM_TASK_H

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

class SetPoseFromTask: public BT::SyncActionNode
{
public:
  SetPoseFromTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
