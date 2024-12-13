#include "project11_navigation/plugins/action/debug_blackboard.h"
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

DebugBlackboard::DebugBlackboard(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");

}

BT::PortsList DebugBlackboard::providedPorts()
{
  return {BT::InputPort<std::string>("value")};
}

BT::NodeStatus DebugBlackboard::tick()
{

  BT::Expected<std::string> value = getInput<std::string>("value");
  if(!value)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "BT Node: " << name() <<  " missing input [value]");
  }
  else
    RCLCPP_INFO_STREAM(node_->get_logger(), "BT Node: " << name() << " value: " << value.value());
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::DebugBlackboard>("DebugBlackboard");
}
