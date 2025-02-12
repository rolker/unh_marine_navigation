#include <project11_navigation/plugins/action/predict_stopping_pose.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace project11_navigation
{

PredictStoppingPose::PredictStoppingPose(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList PredictStoppingPose::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Odometry>("odometry", "{odometry}", "Robot's current odometry state"),
    BT::InputPort<geometry_msgs::msg::Accel>("deceleration", "{robot_default_deceleration}", "Deceleration to use to predict stopping pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "{pose}", "Predicted stopping pose")
  };
}

BT::NodeStatus PredictStoppingPose::tick()
{
  auto odom = getInput<nav_msgs::msg::Odometry>("odometry");
  if(!odom)
  {
    throw BT::RuntimeError(name(), " missing required input [odometry]: ", odom.error() );
  }

  auto decel = getInput<geometry_msgs::msg::Accel>("deceleration");
  if(!decel)
  {
    throw BT::RuntimeError(name(), " missing required input [deceleration]: ", decel.error() );
  }


  tf2::Vector3 motion_vector;
  tf2::fromMsg(odom.value().twist.twist.linear, motion_vector);
  tf2::Quaternion orientation;
  tf2::fromMsg(odom.value().pose.pose.orientation, orientation);
  motion_vector = tf2::quatRotate(orientation, motion_vector);

  double stop_time = -motion_vector.length()/decel.value().linear.x;
  motion_vector *= (stop_time/2.0);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.value().header;
  pose.pose = odom.value().pose.pose;
  pose.pose.position.x += motion_vector.getX();
  pose.pose.position.y += motion_vector.getY();
  pose.pose.position.z += motion_vector.getZ();

  setOutput("pose", pose);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::PredictStoppingPose>("PredictStoppingPose");
}

