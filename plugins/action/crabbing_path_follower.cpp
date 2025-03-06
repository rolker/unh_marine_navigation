#include "project11_navigation/plugins/action/crabbing_path_follower.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "project11/utils.h"
#include "project11/pid.h"
#include "project11_navigation/context.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

namespace project11_navigation
{

CrabbingPathFollower::CrabbingPathFollower(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{
}

BT::PortsList CrabbingPathFollower::providedPorts()
{
  return {
    BT::InputPort<Context::Ptr>("context", "{@context}", "Navigation context"),
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_path", "{navigation_path}", "Path to follow"),
    BT::InputPort<int>("current_navigation_segment", "{current_navigation_segment}", "Current segment of the path"),
    BT::InputPort<double>("target_speed", "{target_speed}", "Target speed"),
    BT::OutputPort<geometry_msgs::msg::TwistStamped>("command_velocity", "{command_velocity}", "Output commanded velocity"),
    BT::BidirectionalPort<std::shared_ptr<project11::PID> >("pid", "{path_follower_pid}", "PID controller for crabbing"),
  };
}

BT::NodeStatus CrabbingPathFollower::onStart()
{
  if(!crab_angle_publisher_)
  {
    auto context = getInput<Context::Ptr>("context");
    if(!context)
    {
      throw BT::RuntimeError(name(), " missing required input [context]: ", context.error() );
    }
    auto node = context.value()->node().lock();
    crab_angle_publisher_ = node->create_publisher<std_msgs::msg::Float32>("crab_angle", 1);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CrabbingPathFollower::onRunning()
{
  auto context = getInput<Context::Ptr>("context");
  if(!context)
  {
    throw BT::RuntimeError(name(), " missing required input [context]: ", context.error() );
  }

  if(!context.value()->robot().enabled())
    return BT::NodeStatus::RUNNING;


  auto navigation_path_bb = getInput<std::shared_ptr<std::vector< geometry_msgs::msg::PoseStamped> > >("navigation_path");
  if(!navigation_path_bb)
  {
    throw BT::RuntimeError("missing required input [navigation_path]: ", navigation_path_bb.error() );
  }
  auto navigation_path = navigation_path_bb.value();

  int segment_count = std::max<int>(0,navigation_path->size()-1);

  auto current_segment = getInput<int>("current_navigation_segment");
  if(!current_segment)
  {
    throw BT::RuntimeError("missing required input [current_navigation_segment]: ", current_segment.error() );
  }

  if(current_segment.value() < 0 || current_segment.value() > segment_count)
    return BT::NodeStatus::FAILURE;

  // We're done if we are at the segment past the last one
  if(current_segment.value() == segment_count)
    return BT::NodeStatus::SUCCESS;


  auto odom = context.value()->robot().odometry();

  auto tf_buffer = context.value()->tfBuffer();

  std::shared_ptr<project11::PID> pid;
  auto pid_bb = getInput<std::shared_ptr<project11::PID> >("pid");
  if(pid_bb)
    pid = pid_bb.value();

  // create pid if it's not on the blackboard yet
  if(!pid)
  {
    auto node = context.value()->node().lock();
    pid = std::make_shared<project11::PID>(node, "path_follower/pid") ;
    setOutput("pid", pid);
  }

  double target_speed = 0.0;
  auto target_speed_bb = getInput<double>("target_speed");
  if(target_speed_bb)
    target_speed = target_speed_bb.value();

  geometry_msgs::msg::TransformStamped base_to_map;
  try
  {
    base_to_map = tf_buffer->lookupTransform(navigation_path->front().header.frame_id , odom.child_frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    auto node = context.value()->node().lock();
    RCLCPP_WARN_STREAM(node->get_logger(), "FollowPathCommand node named " << name() << " Error getting path to base_frame transform: " << ex.what());
    return BT::NodeStatus::FAILURE;
  }

  auto p1 = (*navigation_path)[current_segment.value()];
  auto p2 = (*navigation_path)[current_segment.value()+1];

  auto segment_dx = p2.pose.position.x - p1.pose.position.x;
  auto segment_dy = p2.pose.position.y - p1.pose.position.y;

  project11::AngleRadians segment_azimuth(atan2(segment_dy, segment_dx));
  auto segment_distance = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

  // vehicle distance and azimuth relative to the segment's start point
  double dx = p1.pose.position.x - base_to_map.transform.translation.x;
  double dy = p1.pose.position.y - base_to_map.transform.translation.y;
  auto vehicle_distance = sqrt(dx*dx+dy*dy);

  project11::AngleRadians vehicle_azimuth(atan2(-dy, -dx));

  auto error_azimuth = vehicle_azimuth - segment_azimuth;
     
  auto sin_error_azimuth = sin(error_azimuth);
  auto cos_error_azimuth = cos(error_azimuth);

  // Distance traveled along the line.
  auto progress = vehicle_distance*cos_error_azimuth;

  auto cross_track_error = vehicle_distance*sin_error_azimuth;
  auto crab_angle = project11::AngleDegrees(pid->update(cross_track_error, odom.header.stamp));
  project11::AngleRadians heading(tf2::getYaw(base_to_map.transform.rotation));

  std_msgs::msg::Float32 crab_angle_msg;
  crab_angle_msg.data = crab_angle.value();
  crab_angle_publisher_->publish(crab_angle_msg);

  project11::AngleRadians target_heading = segment_azimuth +	crab_angle;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = odom.header.stamp;
  cmd_vel.header.frame_id = odom.child_frame_id;
  
  cmd_vel.twist.angular.z = project11::AngleRadiansZeroCentered(target_heading-heading).value();

  rclcpp::Time segment_start_time = p1.header.stamp;
  rclcpp::Time segment_end_time = p2.header.stamp;
  if(segment_start_time != rclcpp::Time() && segment_end_time != rclcpp::Time() && segment_end_time > segment_start_time)
  {
    auto dt = segment_end_time - segment_start_time;
    target_speed = segment_distance/dt.seconds();
  }

  double cos_crab = std::max(cos(crab_angle), 0.5);
  cmd_vel.twist.linear.x = target_speed/cos_crab;
  setOutput("command_velocity", cmd_vel);
  return BT::NodeStatus::RUNNING;
}


void CrabbingPathFollower::onHalted()
{
  
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::CrabbingPathFollower>("CrabbingPathFollower");
}
