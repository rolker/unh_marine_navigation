#include "project11_navigation/navigator_settings.h"

namespace project11_navigation
{

NavigatorSettings::NavigatorSettings(rclcpp_lifecycle::LifecycleNode::WeakPtr node_ptr)
{
  auto node = node_ptr.lock();
  node->declare_parameter("waypoint_reached_distance", waypoint_reached_distance);
  node->get_parameter("waypoint_reached_distance", waypoint_reached_distance);

  node->declare_parameter("survey_lead_in_distance", survey_lead_in_distance);
  node->get_parameter("survey_lead_in_distance", survey_lead_in_distance);

  node->declare_parameter("maximum_cross_track_error", maximum_cross_track_error);
  node->get_parameter("maximum_cross_track_error", maximum_cross_track_error);
}

} // namespace project11_navigation
