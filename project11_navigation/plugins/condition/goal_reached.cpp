#include "project11_navigation/plugins/condition/goal_reached.h"
#include "nav_msgs/msg/odometry.hpp"
#include "project11_navigation/utilities.h"
#include "nav2_util/robot_utils.hpp"

namespace project11_navigation
{

GoalReached::GoalReached(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{

}

BT::PortsList GoalReached::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "{goal_pose}", "Pose to compare current state with"),
    BT::InputPort<double>("heading_accuracy", "180", "How much can the heading can vary in degrees. 180 means it doesn't matter."),
    BT::InputPort<double>("goal_reached_distance", "1.0", "Distance to goal to be considered reached."),
  };
}

BT::NodeStatus GoalReached::tick()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto robot_base_frame = node->get_parameter("robot_base_frame").as_string();
  auto transform_tolerance = node->get_parameter("transform_tolerance").as_double();

  auto goal_bb = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if(!goal_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [goal_pose]: ", goal_bb.error() );
  }

  const auto& goal = goal_bb.value();

  if(!tf_)
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal.header.frame_id, robot_base_frame, transform_tolerance))
  {
    RCLCPP_DEBUG(node->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  auto distance_bb = getInput<double>("goal_reached_distance");
  if(!distance_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [goal_reached_distance]: ", distance_bb.error() );
  }

  auto distance = distance_bb.value();

  auto odom_at_surface = current_pose.pose;
  odom_at_surface.position.z = 0;
  if(length(vectorBetween(odom_at_surface, goal.pose)) <= distance)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation
