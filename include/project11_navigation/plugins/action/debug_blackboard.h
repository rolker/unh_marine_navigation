#ifndef PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H
#define PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

template <typename T>
class DebugBlackboard: public BT::SyncActionNode
{
public:
  DebugBlackboard(const std::string& name, const BT::NodeConfig& config):
    BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
      return {
        BT::InputPort<std::string>("message", "message", "Message to print, such as the variable name"),
        BT::InputPort<T>("value", "Value to print in the ROS log")
      };

  }

  BT::NodeStatus tick() override
  {
    BT::Expected<std::string> message = getInput<std::string>("message");
    std::string message_string = "Debug: ";
    if(message)
    {
      message_string = message.value();
    }
    BT::Expected<T> value = getInput<T>("value");
    if(!value)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "BT Node: " << name() << " " << message_string << " missing input [value]");
    }
    else
      RCLCPP_INFO_STREAM(node_->get_logger(), "BT Node: " << name() << " " << message_string << " value: " << value.value());
    return BT::NodeStatus::SUCCESS;
}

private:
  rclcpp::Node::SharedPtr node_;

};

using DebugBlackboardDouble = DebugBlackboard<double>;
using DebugBlackboardString = DebugBlackboard<std::string>;


} // namespace project11_navigation

#endif
