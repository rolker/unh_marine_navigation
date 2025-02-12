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
    BT::InputPort<nav_msgs::msg::Odometry>("odometry", "{odometry}", "Robot's current odometry state"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "{goal_pose}", "Pose to compare current state with"),
    BT::InputPort<double>("waypoint_reached_distance", "{waypoint_reached_distance}", "Distance in meters to consider the waypoint reached"),
    BT::InputPort<double>("heading_accuracy", "180", "How much can the heading can vary in degrees. 180 means it doesn't matter.")
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
