#include "project11_navigation/plugins/condition/goal_reached.h"
#include "nav_msgs/msg/odometry.hpp"
#include "project11_navigation/utilities.h"
#include "project11_navigation/context.h"

namespace project11_navigation
{

GoalReached::GoalReached(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{

}

BT::PortsList GoalReached::providedPorts()
{
  return {
    BT::InputPort<Context::Ptr>("context", "{@context}", "Navigation context"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "{goal_pose}", "Pose to compare current state with"),
    BT::InputPort<double>("heading_accuracy", "180", "How much can the heading can vary in degrees. 180 means it doesn't matter.")
  };
}

BT::NodeStatus GoalReached::tick()
{
  auto context_bb = getInput<Context::Ptr>("context");
  if(!context_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [context]: ", context_bb.error() );
  }
  auto context = context_bb.value();

  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if(!goal)
  {
    throw BT::RuntimeError(name(), " missing required input [goal_pose]: ", goal.error() );
  }

  auto odom = context->robot().odometry();
  auto distance = context->navigator_settings().waypoint_reached_distance;

  auto odom_at_surface = odom.pose.pose;
  odom_at_surface.position.z = 0;
  if(length(vectorBetween(odom_at_surface, goal.value().pose)) <= distance)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::GoalReached>("GoalReachedCondition");
}
