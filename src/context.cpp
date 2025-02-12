#include "project11_navigation/context.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace project11_navigation
{

Context::Context(rclcpp_lifecycle::LifecycleNode::WeakPtr node_ptr):
  environment_(node_ptr),
  robot_(node_ptr),
  robot_capabilities_(node_ptr),
  node_(node_ptr)
{
  auto node = node_.lock();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

const Environment& Context::environment() const
{
  return environment_;
}

const Robot& Context::robot() const
{
  return robot_;
}

Robot& Context::robot()
{
  return robot_;
}

const RobotCapabilities& Context::robot_capabilities() const
{
  return robot_capabilities_;
}


rclcpp_lifecycle::LifecycleNode::WeakPtr Context::node() const
{
  return node_;
}

std::shared_ptr<tf2_ros::Buffer> Context::tfBuffer() const
{
  return tf_buffer_;
}

geometry_msgs::msg::PoseStamped Context::getPoseInFrame(std::string frame_id)
{
  auto odom = robot_.odometry();
  geometry_msgs::msg::PoseStamped ret;
  ret.header = odom.header;
  ret.pose = odom.pose.pose;
  try
  {
    if(ret.header.frame_id != frame_id)
    {
      tf_buffer_->transform(ret, frame_id, tf2::durationFromSec(0.25));
      ret.header.frame_id = frame_id;
    }
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_STREAM(node_.lock()->get_logger(), "Context::getPoseInFrame " << ex.what());
  }

  return ret;
}

} // namespace project11_navigation
