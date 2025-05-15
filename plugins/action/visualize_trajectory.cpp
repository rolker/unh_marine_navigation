#include "project11_navigation/plugins/action/visualize_trajectory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <project11_navigation/bt_types.h>
#include "project11_navigation/context.h"

namespace project11_navigation
{

VisualizeTrajectory::VisualizeTrajectory(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
}

BT::PortsList VisualizeTrajectory::providedPorts()
{
  return {
    BT::InputPort<Context::Ptr>("context", "{@context}", "Navigation context"),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped> >("trajectory", "{navigation_trajectory}", "Trajectory to visualize"),
    BT::InputPort<std::shared_ptr<visualization_msgs::msg::MarkerArray> >("marker_array", "{marker_array}", "Pointer to MarkerArray to add visualization markers to"),
    BT::InputPort<std::string>("namespace", "trajectory", "Used in ns field of Markers"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose", "{current_pose}", "Current pose of the robot"),
    BT::InputPort<double>("scale", "1.0", "Display size"),
    BT::InputPort<std_msgs::msg::ColorRGBA>("past_color", "0.25, 0.25, 0.25, 0.5", "Color for past segments"),
    BT::InputPort<std_msgs::msg::ColorRGBA>("current_color", "0.35, 0.35, 0.5, 0.75", "Color for current segment"),
    BT::InputPort<std_msgs::msg::ColorRGBA>("future_color", ".25, .25, .4, .5", "Color for future segments"),
    BT::InputPort<int>("current_segment", "{current_navigation_segment}", "Index of the current segment of the trajectory"),
  };
}

BT::NodeStatus VisualizeTrajectory::tick()
{
  auto context = getInput<Context::Ptr>("context");
  if(!context)
  {
    throw BT::RuntimeError(name(), " missing required input [context]: ", context.error() );
  }
  auto node = context.value()->node().lock();

  auto trajectory = getInput<std::vector<geometry_msgs::msg::PoseStamped> >("trajectory");
  if(!trajectory)
  {
    throw BT::RuntimeError(name(), " missing required input [trajectory]: ", trajectory.error() );
  }

  auto marker_array = getInput<std::shared_ptr<visualization_msgs::msg::MarkerArray> >("marker_array");
  if(!marker_array)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "VisualizeTrajectory node named " << name() << " missing [marker_array]");
    return BT::NodeStatus::FAILURE;
  }
  if(!marker_array.value())
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "VisualizeTrajectory node named " << name() << " [marker_array] is null");
    return BT::NodeStatus::FAILURE;
  }

  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  if(!current_pose)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "VisualizeTrajectory node named " << name() << " missing [current_pose]");
    return BT::NodeStatus::FAILURE;
  }

  auto current_segment = getInput<int>("current_segment");
  if(!current_segment)
  {
    throw BT::RuntimeError(name(), " missing required input [current_segment]: ", current_segment.error() );
  }

  auto ns = getInput<std::string>("namespace");

  auto scale = getInput<double>("scale");

  std::vector<std::string> color_labels = {
    "past_color", "current_color", "future_color",
  };

  std::vector<BT::Expected<std_msgs::msg::ColorRGBA> > colors;
  for(auto label: color_labels)
    colors.push_back(getInput<std_msgs::msg::ColorRGBA>(label));

  if(!trajectory.value().empty())
  {
    const auto& poses = trajectory.value();

    std::vector<visualization_msgs::msg::Marker> markers(3);
    for(int i = 0; i < markers.size(); i++)
    {
      markers[i].header.frame_id = poses.front().header.frame_id;
      markers[i].header.stamp = current_pose.value().header.stamp;
      markers[i].id = i;
      markers[i].ns = ns.value();
      markers[i].action = visualization_msgs::msg::Marker::ADD;
      markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
      markers[i].pose.orientation.w = 1.0;
      markers[i].color = colors[i].value();
      markers[i].scale.x = scale.value();
      markers[i].lifetime = rclcpp::Duration::from_seconds(2.0);
    }

    int markers_index = 0; // start with past
    for(int i = 0; i < poses.size()-1; i++)
    {
      // still working on past markers?
      if(markers_index == 0)
      {
        // did we reach the current segment?
        if(i == current_segment.value())
        {
          // add the final point if necessary
          if(!markers[0].points.empty())
            markers[0].points.push_back(poses[i].pose.position);
          // add current segment
          markers[1].points.push_back(poses[i].pose.position);
          markers[1].points.push_back(poses[i+1].pose.position);
          // rest will be assigned to future
          markers_index = 2; 
          continue; // so we don't add to future on this iteration
        }
      }
      markers[markers_index].points.push_back(poses[i].pose.position);
    }
    // add last position to complete the segment, if necessary
    if(!markers[markers_index].points.empty())
      markers[markers_index].points.push_back(poses.back().pose.position);

    for(const auto& marker: markers)
      marker_array.value()->markers.push_back(marker);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation
