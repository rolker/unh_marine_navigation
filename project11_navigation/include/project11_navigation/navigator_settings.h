#ifndef PROJECT11_NAVIGATION_NAVIGATOR_SETTINGS_H
#define PROJECT11_NAVIGATION_NAVIGATOR_SETTINGS_H

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace project11_navigation
{

/// Parameters used for navigation
struct NavigatorSettings
{
  NavigatorSettings(rclcpp_lifecycle::LifecycleNode::WeakPtr node_ptr);

  /// Distance in meters within which a waypoint is deemed reached
  double waypoint_reached_distance = 10.0;

  /// Distance in meters to extend the start of a survey line to allow heading to settle
  double survey_lead_in_distance = 2.0;

  /// Maximum cross track error in meters beyond which the robot may be considered off course
  double maximum_cross_track_error = 5.0;
};

}

#endif
