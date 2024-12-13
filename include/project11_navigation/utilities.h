#ifndef PROJECT11_NAVIGATION_UTILITIES_H
#define PROJECT11_NAVIGATION_UTILITIES_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace project11_navigation
{

// Read a number from the parameter server as either a double or int.
// This avoids crashes when trying to read a parameter without a decimal point as a double.
// double readDoubleOrIntParameter(ros::NodeHandle &nh, const std::string& parameter, double default_value);


/// Declare parameters for the linear and angular components of a type.
template<typename T>
void declareLinearAngularParameters(rclcpp::Node::SharedPtr node, const std::string& parameter, T& value, const T& default_value)
{
  node->declare_parameter(parameter+"linear/x", default_value.linear.x);
  node->get_parameter(parameter+"linear/x", value.linear.x);
  node->declare_parameter(parameter+"linear/y", default_value.linear.y);
  node->get_parameter(parameter+"linear/y", value.linear.y);
  node->declare_parameter(parameter+"linear/z", default_value.linear.z);
  node->get_parameter(parameter+"linear/z", value.linear.z);

  node->declare_parameter(parameter+"angular/x", default_value.angular.x);
  node->get_parameter(parameter+"angular/x", value.angular.x);
  node->declare_parameter(parameter+"angular/y", default_value.angular.y);
  node->get_parameter(parameter+"angular/y", value.angular.y);
  node->declare_parameter(parameter+"angular/z", default_value.angular.z);
  node->get_parameter(parameter+"angular/z", value.angular.z);
}

// {
//   value.linear.x = readDoubleOrIntParameter(nh, parameter+"/linear/x", default_value.linear.x);
//   value.linear.y = readDoubleOrIntParameter(nh, parameter+"/linear/y", default_value.linear.y);
//   value.linear.z = readDoubleOrIntParameter(nh, parameter+"/linear/z", default_value.linear.z);
//   value.angular.x = readDoubleOrIntParameter(nh, parameter+"/angular/x", default_value.angular.x);
//   value.angular.y = readDoubleOrIntParameter(nh, parameter+"/angular/y", default_value.angular.y);
//   value.angular.z = readDoubleOrIntParameter(nh, parameter+"/angular/z", default_value.angular.z);
// }

// Adjust the timestamps along a trajectory for the given constant speed and starting
// at the first pose's timestamp.
void adjustTrajectoryForSpeed(std::vector<geometry_msgs::msg::PoseStamped>& trajectory, double speed);


void adjustPathOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path);

geometry_msgs::msg::Vector3 vectorBetween(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to);
double length(const geometry_msgs::msg::Vector3& vector);
geometry_msgs::msg::Vector3 normalize(const geometry_msgs::msg::Vector3& vector);

} // namespace project11_navigation

#endif
