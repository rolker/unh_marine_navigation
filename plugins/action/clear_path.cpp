#include <project11_navigation/plugins/action/clear_path.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace project11_navigation
{

ClearPath::ClearPath(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList ClearPath::providedPorts()
{
  return {
    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_path")
  };
}

BT::NodeStatus ClearPath::tick()
{

  setOutput("navigation_path", std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> >());
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::ClearPath>("ClearPath");
}
