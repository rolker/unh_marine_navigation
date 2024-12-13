#include <project11_navigation/plugins/action/generate_plan.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

extern "C" {
#include "dubins_curves/dubins.h"
};


namespace project11_navigation
{

GeneratePlan::GeneratePlan(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList GeneratePlan::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
    BT::InputPort<std::string>("planner"),
    BT::InputPort<double>("turn_radius"),
    BT::InputPort<double>("lead_in_distance"),
    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_trajectory"),
    BT::OutputPort<int>("current_navigation_segment")
  };
}

BT::NodeStatus GeneratePlan::tick()
{
  auto start_pose = getInput<geometry_msgs::msg::PoseStamped>("start_pose");
  if(!start_pose)
  {
    throw BT::RuntimeError(name(), " missing required input [start_pose]: ", start_pose.error() );
  }

  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if(!goal_pose)
  {
    throw BT::RuntimeError(name(), " missing required input [goal_pose]: ", goal_pose.error() );
  }

  auto turn_radius = getInput<double>("turn_radius");
  if(!turn_radius)
  {
    throw BT::RuntimeError(name(), " missing required input [turn_radius]: ", turn_radius.error() );
  }

  auto lead_in_distance = getInput<double>("lead_in_distance");
  if(!lead_in_distance)
  {
    throw BT::RuntimeError(name(), " missing required input [lead_in_distance]: ", lead_in_distance.error() );
  }


  double start[3];
  start[0] = start_pose.value().pose.position.x;
  start[1] = start_pose.value().pose.position.y;
  start[2] = tf2::getYaw(start_pose.value().pose.orientation);
  
  double target[3];
  target[0] = goal_pose.value().pose.position.x;
  target[1] = goal_pose.value().pose.position.y;
  target[2] = tf2::getYaw(goal_pose.value().pose.orientation);

  if(lead_in_distance.value() > 0.0)
  {
    auto cos_yaw = cos(target[2]);
    auto sin_yaw = sin(target[2]);
    target[0] -= lead_in_distance.value()*cos_yaw;
    target[1] -= lead_in_distance.value()*sin_yaw;
  }

  DubinsPath path;

  if(dubins_shortest_path(&path, start, target, turn_radius.value()) == 0)
  {
    auto path_length = dubins_path_length(&path);
    auto step_size = turn_radius.value() / 5.0;
    auto nav_trajectory = std::make_shared<std::vector<geometry_msgs::msg::PoseStamped> >();
    nav_trajectory->push_back(start_pose.value());
    double current_length = step_size;
    while(current_length < path_length)
    {
      double q[3];
      if(dubins_path_sample(&path, current_length, q) != 0)
        break;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = goal_pose.value().header.frame_id;
      pose.pose.position.x = q[0];
      pose.pose.position.y = q[1];
      tf2::Quaternion quat;
      quat.setRPY(0,0,q[2]);
      pose.pose.orientation = tf2::toMsg(quat);
      nav_trajectory->push_back(pose);
      current_length += step_size;
    }
    nav_trajectory->push_back(goal_pose.value());
    setOutput("navigation_trajectory", nav_trajectory);
    setOutput("current_navigation_segment", 0);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::GeneratePlan>("GeneratePlan");
}
