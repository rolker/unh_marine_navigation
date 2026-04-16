#include "marine_nav_behavior_tree/plugins/action/path_to_pose_vector.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace marine_nav_behavior_tree
{

PathToPoseVector::PathToPoseVector(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
}

BT::PortsList PathToPoseVector::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "{path}", "Path to convert to pose vector"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped> >("poses", "{poses}", "Vector of poses to set")
  };
}

BT::NodeStatus PathToPoseVector::tick()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  if(!path)
  {
    throw BT::RuntimeError(name(), " missing required input [path]: ", path.error() );
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for(const auto& pose: path.value().poses)
  {
    auto p = pose;
    p.header.stamp = builtin_interfaces::msg::Time();   // NEW: zero stamp = "latest" in TF lookups
    poses.push_back(p);
  }

  setOutput("poses", poses);

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree
