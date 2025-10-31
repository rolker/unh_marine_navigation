#include "project11_navigation/plugins/action/pose_vector_to_path.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace project11_navigation
{

PoseVectorToPath::PoseVectorToPath(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
}

BT::PortsList PoseVectorToPath::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped> >("poses", "{poses}", "Vector of poses to convert to path"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{path}", "Path to set")
  };
}

BT::NodeStatus PoseVectorToPath::tick()
{
  auto poses = getInput<std::vector<geometry_msgs::msg::PoseStamped> >("poses");
  if(!poses)
  {
    throw BT::RuntimeError(name(), " missing required input [poses]: ", poses.error() );
  }

  nav_msgs::msg::Path path;
  if(poses.value().empty())
  {
    setOutput("path", path);
    return BT::NodeStatus::SUCCESS;
  }
  
  for(const auto& pose: poses.value())
  {
    path.poses.push_back(pose);
  }
  if(!path.poses.empty())
  {
    path.header = path.poses.front().header;
  }

  setOutput("path", path);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation
