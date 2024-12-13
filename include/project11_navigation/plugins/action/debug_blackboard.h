#ifndef PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H
#define PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

class DebugBlackboard: public BT::SyncActionNode
{
public:
  DebugBlackboard(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;

};

} // namespace project11_navigation

#endif
