#include "project11_navigation/plugins/action/update_state.h"
#include <project11_navigation/context.h>
#include <project11_navigation/robot.h>
#include <project11_navigation/robot_capabilities.h>
#include <nav_msgs/msg/odometry.hpp>

namespace project11_navigation
{

UpdateState::UpdateState(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList UpdateState::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Context> >("context", "{context}", "Navigation context"),
    BT::OutputPort<std::string>("piloting_mode", "{piloting_mode}", "Current Project11 piloting mode"),
    BT::OutputPort<std::string>("base_frame", "{base_frame}", "Base frame_id for the robot for use with cmd_vel"),
    BT::OutputPort<nav_msgs::msg::Odometry>("odometry", "{odometry}", "Robot's current odometry state"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose", "{current_pose}", "Robot's current pose"),
    BT::OutputPort<geometry_msgs::msg::TwistStamped>("command_velocity", "{command_velocity}", "Initial command set to 0"),
    BT::OutputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer", "{tf_buffer}", "Transform buffer"),
    BT::OutputPort<std::shared_ptr<visualization_msgs::msg::MarkerArray> >("marker_array", "{marker_array}", "Marker array to add visualization markers to"),
    BT::InputPort<geometry_msgs::msg::Polygon>("robot_footprint", "{robot_footprint}", "Footprint of the robot"),
    BT::OutputPort<std::shared_ptr<OccupancyGrid> >("local_costmap", "{local_costmap}", "Local costmap")
  };
}

BT::NodeStatus UpdateState::tick()
{
  auto context = getInput<std::shared_ptr<Context> >("context");

  if(context.value()->robot().enabled())
    setOutput("piloting_mode", "autonomous");
  else
    setOutput("piloting_mode", "not_autonomous");

  setOutput("base_frame", context.value()->robot().baseFrame());
  setOutput("odometry", context.value()->robot().odometry());
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = context.value()->robot().odometry().header;
  current_pose.pose = context.value()->robot().odometry().pose.pose;
  setOutput("current_pose", current_pose);
  geometry_msgs::msg::TwistStamped ts;
  ts.header.frame_id = context.value()->robot().baseFrame();
  setOutput("command_velocity", ts);
  setOutput("tf_buffer", context.value()->tfBuffer());

  auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
  auto footprint = getInput<geometry_msgs::msg::Polygon>("robot_footprint");
  if(footprint)
  {
    context.value()->robot().updateMarkers(*marker_array, footprint.value());
  }
  setOutput("marker_array", marker_array);

  setOutput("local_costmap", context.value()->environment().localCostmap());

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::UpdateState>("UpdateState");
}

