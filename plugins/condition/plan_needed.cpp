#include "project11_navigation/plugins/condition/plan_needed.h"
#include "nav_msgs/msg/odometry.hpp"
#include "project11_navigation/utilities.h"

namespace project11_navigation
{

PlanNeeded::PlanNeeded(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{

}

BT::PortsList PlanNeeded::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_trajectory"),
    BT::InputPort<double>("cross_track_error"),
    BT::InputPort<double>("maximum_cross_track_error"),
  };
}

BT::NodeStatus PlanNeeded::tick()
{
  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if(!goal_pose)
  {
    throw BT::RuntimeError("PlanNeeded node named ", name(), " missing required input [goal_pose]: ", goal_pose.error() );
  }

  auto nav_trajectory = getInput<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_trajectory");

  auto cross_track_error = getInput<double>("cross_track_error");
  if(!cross_track_error)
  {
    throw BT::RuntimeError("PlanNeeded node named ", name(), " missing required input [cross_track_error]: ", cross_track_error.error() );
  }

  auto maximum_cross_track_error = getInput<double>("maximum_cross_track_error");
  if(!maximum_cross_track_error)
  {
    throw BT::RuntimeError("PlanNeeded node named ", name(), " missing required input [maximum_cross_track_error]: ", maximum_cross_track_error.error() );
  }

  if(fabs(cross_track_error.value()) > maximum_cross_track_error.value())
    return BT::NodeStatus::SUCCESS;

  if(nav_trajectory && nav_trajectory.value() && !nav_trajectory.value()->empty())
  {
    if(goal_pose.value() != nav_trajectory.value()->back())
      return BT::NodeStatus::SUCCESS;



    
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::PlanNeeded>("PlanNeededCondition");
}
