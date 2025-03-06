#ifndef PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H
#define PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "project11_navigation/context.h"

namespace project11_navigation
{

template <typename T>
class DebugBlackboard: public BT::SyncActionNode
{
public:
  DebugBlackboard(const std::string& name, const BT::NodeConfig& config):
    BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
      return {
        BT::InputPort<Context::Ptr>("context", "{@context}", "Navigation context"),
        BT::InputPort<std::string>("message", "message", "Message to print, such as the variable name"),
        BT::InputPort<T>("value", "Value to print in the ROS log")
      };

  }

  BT::NodeStatus tick() override
  {
    auto context = getInput<Context::Ptr>("context");
    if(!context)
    {
      throw BT::RuntimeError(name(), " missing required input [context]: ", context.error() );
    }
    auto node = context.value()->node().lock();

    BT::Expected<std::string> message = getInput<std::string>("message");
    std::string message_string = "Debug: ";
    if(message)
    {
      message_string = message.value();
    }
    BT::Expected<T> value = getInput<T>("value");
    if(!value)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "BT Node: " << name() << " " << message_string << " missing input [value]");
    }
    else
      RCLCPP_INFO_STREAM(node->get_logger(), "BT Node: " << name() << " " << message_string << " value: " << value.value());
    return BT::NodeStatus::SUCCESS;
}

};

using DebugBlackboardDouble = DebugBlackboard<double>;
using DebugBlackboardString = DebugBlackboard<std::string>;


} // namespace project11_navigation

#endif
