#include <project11_navigation/plugins/action/clear_path.h>
#include "nav_msgs/msg/path.hpp"

namespace project11_navigation
{

ClearPath::ClearPath(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList ClearPath::providedPorts()
{
  return {
    BT::OutputPort<nav_msgs::msg::Path>("navigation_path", "{navigation_path}", "Empty shared pointer to a path to follow")
  };
}

BT::NodeStatus ClearPath::tick()
{

  setOutput("navigation_path", nav_msgs::msg::Path());
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

