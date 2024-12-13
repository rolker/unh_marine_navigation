#ifndef PROJECT11_NAVIGATION_ACTIONS_ROBOT_CAPABILITIES_LOADER_H
#define PROJECT11_NAVIGATION_ACTIONS_ROBOT_CAPABILITIES_LOADER_H

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

class RobotCapabilitiesLoader: public BT::SyncActionNode
{
public:
  RobotCapabilitiesLoader(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace project11_navigation

#endif
