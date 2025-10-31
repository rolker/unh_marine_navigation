#ifndef PROJECT11_NAVIGATION_PLATFORM_H
#define PROJECT11_NAVIGATION_PLATFORM_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace project11_navigation
{

/// Represents a platfrom that can be tracked in ROS using Odometry messages.
class Platform
{
public:
  Platform(rclcpp_lifecycle::LifecycleNode::WeakPtr node);
  std::string baseFrame() const;
  const nav_msgs::msg::Odometry &odometry() const;

  void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr& msg);

protected:
  nav_msgs::msg::Odometry odom_;

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

} // namespace project11_navigation

#endif
