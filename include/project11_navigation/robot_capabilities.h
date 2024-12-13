#ifndef PROJECT11_NAVIGATION_ROBOT_CAPABILITIES_H
#define PROJECT11_NAVIGATION_ROBOT_CAPABILITIES_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"

namespace project11_navigation
{

/// Motion limits of a robot
struct RobotCapabilities
{
  RobotCapabilities(rclcpp::Node::SharedPtr node);

  geometry_msgs::msg::Twist min_velocity;
  geometry_msgs::msg::Twist max_velocity;

  /// default cruise speed
  geometry_msgs::msg::Twist default_velocity;

  /// Max available acceleration
  geometry_msgs::msg::Accel max_acceleration;

  /// Default acceleration that should be used
  /// for normal operations
  geometry_msgs::msg::Accel default_acceleration;

  /// Powered deceleration
  geometry_msgs::msg::Accel max_deceleration;

  /// Drifting deceleration
  geometry_msgs::msg::Accel default_deceleration;

  /// Map speed to turn radius for a Dubin's robot.
  /// If empty, robot can turn in place so radius is 0.
  /// With only one entry, turn radius is constant in speed range.
  /// Speeds at which the robot can't run can be indicated with a NaN.
  std::map<double, double> turn_radius_map;
  double getTurnRadiusAtSpeed(double speed) const;
  double turn_radius;

  /// Outline in XY plane of the robot
  //std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::msg::Polygon footprint;

  /// Radius used for collision checking
  double radius;
};

}

#endif
