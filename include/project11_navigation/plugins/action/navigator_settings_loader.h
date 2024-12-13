#ifndef PROJECT11_NAVIGATION_ACTIONS_NAVIGATOR_SETTINGS_LOADER_H
#define PROJECT11_NAVIGATION_ACTIONS_NAVIGATOR_SETTINGS_LOADER_H

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

class NavigatorSettingsLoader: public BT::SyncActionNode
{
public:
  NavigatorSettingsLoader(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace project11_navigation

#endif
