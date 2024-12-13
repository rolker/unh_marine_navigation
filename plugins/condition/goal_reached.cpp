#include "project11_navigation/plugins/condition/goal_reached.h"
#include "nav_msgs/msg/odometry.hpp"
#include "project11_navigation/utilities.h"

namespace project11_navigation
{

GoalReached::GoalReached(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{

}

BT::PortsList GoalReached::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Odometry>("odometry"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
    BT::InputPort<double>("waypoint_reached_distance"),
    BT::InputPort<double>("heading_accuracy")
  };
}

BT::NodeStatus GoalReached::tick()
{
  auto odom = getInput<nav_msgs::msg::Odometry>("odometry");
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  auto distance = getInput<double>("waypoint_reached_distance");

  if(odom && goal && distance)
  {
    auto odom_at_surface = odom.value().pose.pose;
    odom_at_surface.position.z = 0;
    if(length(vectorBetween(odom_at_surface, goal.value().pose)) <= distance.value())
      return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::GoalReached>("GoalReachedCondition");
}
