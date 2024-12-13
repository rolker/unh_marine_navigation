#ifndef PROJECT11_NAVIGATION_ROBOT_H
#define PROJECT11_NAVIGATION_ROBOT_H

#include "project11_navigation/platform.h"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"

namespace project11_navigation
{
class RobotCapabilities;

// Represents a platform that can be commanded using ROS Twist messages.
class Robot: public Platform
{
public:
  Robot(rclcpp_lifecycle::LifecycleNode::WeakPtr node);

  /// Sends command to robot if enabled.
  void sendControls(const geometry_msgs::msg::TwistStamped& cmd_vel) const;

  /// Draws robot footprint using visualization markers
  ///  \todo move to a BT Action with access to footprint
  void updateMarkers(visualization_msgs::msg::MarkerArray& marker_array, const geometry_msgs::msg::Polygon& footprint) const;

  
  /// Returns true if robot can accept drive commands.
  bool enabled() const;

private:
  /// Velocity commands to move the robot.
  /// The output of the Navigator.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  /// Controls the output of drive commands.
  /// If true, we are in an autonomous mode where drive commands should be issued.
  /// If false, we are in a mode where drive commands should not be sent, but
  /// planning should occur and results made available for display in a UI to give 
  /// idea of what might happen when switching to an autonomous mode.
  bool enabled_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  void enableCallback(const std_msgs::msg::Bool::UniquePtr& msg);
};

} // namespace project11_navigation

#endif
