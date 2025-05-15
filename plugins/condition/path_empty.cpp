#include "project11_navigation/plugins/condition/path_empty.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace project11_navigation
{
PathEmpty::PathEmpty(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{
}

BT::PortsList PathEmpty::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped> >("path", "{path}", "Path to check")
  };
}

BT::NodeStatus PathEmpty::tick()
{
  auto path = getInput<std::vector<geometry_msgs::msg::PoseStamped> >("path");
  if(!path)
  {
    throw BT::RuntimeError("PathEmpty node named ", name(), " missing required input [path]: ", path.error() );
  }

  if(path.value().empty())
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation
