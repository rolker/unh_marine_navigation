#ifndef MARINE_NAV_UTILITIES_UTILITIES_H
#define MARINE_NAV_UTILITIES_UTILITIES_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace marine_nav_utilities
{


template <typename A>
double quaternionToHeadingDegrees(const A& a)
{
  return 90.0-180.0*tf2::getYaw(a)/M_PI;
}

template <typename A>
double quaternionToHeadingRadians(const A& a)
{
  return (M_PI/2.0)-tf2::getYaw(a);
}

template <typename T>
inline double speedOverGround(const T & v)
{
  tf2::Vector3 v3;
  tf2::fromMsg(v, v3);
  return v3.length();
}


/// Declare parameters for the linear and angular components of a type.
template<typename T, typename NodePtrType>
void declareLinearAngularParameters(NodePtrType node, const std::string& parameter, T& value, const T& default_value)
{
  node->declare_parameter(parameter+".linear.x", default_value.linear.x);
  node->get_parameter(parameter+".linear.x", value.linear.x);
  node->declare_parameter(parameter+".linear.y", default_value.linear.y);
  node->get_parameter(parameter+".linear.y", value.linear.y);
  node->declare_parameter(parameter+".linear.z", default_value.linear.z);
  node->get_parameter(parameter+".linear.z", value.linear.z);

  node->declare_parameter(parameter+".angular.x", default_value.angular.x);
  node->get_parameter(parameter+".angular.x", value.angular.x);
  node->declare_parameter(parameter+".angular.y", default_value.angular.y);
  node->get_parameter(parameter+".angular.y", value.angular.y);
  node->declare_parameter(parameter+".angular.z", default_value.angular.z);
  node->get_parameter(parameter+".angular.z", value.angular.z);
}


/// Adjust the timestamps along a trajectory for the given constant speed and starting
/// at the first pose's timestamp.
void adjustTrajectoryForSpeed(
  std::vector<geometry_msgs::msg::PoseStamped>& trajectory,
  double speed
);

/// Check if quaternion length is about 1.0
bool quaternionSeemsValid(const geometry_msgs::msg::Quaternion& q);

bool quaternionSeemsValid(const tf2::Quaternion& q);


void adjustPathOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, bool only_if_invalid = false);

/// Return the vector from the'from' to 'to'.
template<typename T>
tf2::Vector3 vectorBetween(const T& from_position, const T& to_position)
{
  tf2::Vector3 from, to;
  fromMsg(from_position, from);
  fromMsg(to_position, to);
  return to - from;
}

template<>
tf2::Vector3 vectorBetween(const geometry_msgs::msg::Pose& from_pose, const geometry_msgs::msg::Pose& to_pose);



} // namespace marine_nav_utilities

#endif
