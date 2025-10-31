#include <project11_navigation/platform.h>

namespace project11_navigation
{

Platform::Platform(rclcpp_lifecycle::LifecycleNode::WeakPtr node_ptr)
{
  auto node = node_ptr.lock();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](const nav_msgs::msg::Odometry::UniquePtr &msg){this->odometryCallback(msg);});
}

void Platform::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr& msg)
{
  odom_ = *msg;
}

std::string Platform::baseFrame() const
{
  return odom_.child_frame_id;
}

const nav_msgs::msg::Odometry& Platform::odometry() const
{
  return odom_;
}

} // namespace project11_navigation
