#include <project11_navigation/plugins/action/predict_stopping_pose.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "project11_navigation/context.h"

namespace project11_navigation
{

PredictStoppingPose::PredictStoppingPose(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList PredictStoppingPose::providedPorts()
{
  return {
    BT::InputPort<Context::Ptr>("context", "{@context}", "Navigation context"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "{pose}", "Predicted stopping pose")
  };
}

BT::NodeStatus PredictStoppingPose::tick()
{
  auto context_bb = getInput<Context::Ptr>("context");
  if(!context_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [context]: ", context_bb.error() );
  }
  auto context = context_bb.value();

  auto odom = context->robot().odometry();
  auto decel = context->robot_capabilities().default_deceleration;

  tf2::Vector3 motion_vector;
  tf2::fromMsg(odom.twist.twist.linear, motion_vector);
  tf2::Quaternion orientation;
  tf2::fromMsg(odom.pose.pose.orientation, orientation);
  motion_vector = tf2::quatRotate(orientation, motion_vector);

  double stop_time = -motion_vector.length()/decel.linear.x;
  motion_vector *= (stop_time/2.0);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  pose.pose.position.x += motion_vector.getX();
  pose.pose.position.y += motion_vector.getY();
  pose.pose.position.z += motion_vector.getZ();

  setOutput("pose", pose);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation
