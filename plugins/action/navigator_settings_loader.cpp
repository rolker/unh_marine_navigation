#include "project11_navigation/plugins/action/navigator_settings_loader.h"
#include <project11_navigation/navigator_settings.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

NavigatorSettingsLoader::NavigatorSettingsLoader(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");

  NavigatorSettings nav_settings;
  node_->declare_parameter("waypoint_reached_distance", nav_settings.waypoint_reached_distance);
  node_->declare_parameter("survey_lead_in_distance", nav_settings.survey_lead_in_distance);
  node_->declare_parameter("maximum_cross_track_error", nav_settings.waypoint_reached_distance);
}

BT::PortsList NavigatorSettingsLoader::providedPorts()
{
  return {
    BT::OutputPort<double>("waypoint_reached_distance"),
    BT::OutputPort<double>("survey_lead_in_distance"),
    BT::OutputPort<double>("maximum_cross_track_error"),
  };
}

BT::NodeStatus NavigatorSettingsLoader::tick()
{
  NavigatorSettings nav_settings;

  node_->get_parameter("waypoint_reached_distance", nav_settings.waypoint_reached_distance);

  node_->get_parameter("survey_lead_in_distance", nav_settings.survey_lead_in_distance);

  double max_cross_track;
  node_->get_parameter("maximum_cross_track_error", max_cross_track);

  setOutput("waypoint_reached_distance", nav_settings.waypoint_reached_distance);
  setOutput("survey_lead_in_distance", nav_settings.survey_lead_in_distance);
  setOutput("maximum_cross_track_error", max_cross_track);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::NavigatorSettingsLoader>("NavigatorSettingsLoader");
}
