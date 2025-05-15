#include "project11_navigation/plugins/controllers/crabbing_path_follower.h"
#include "project11/utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace project11_navigation
{
namespace controllers
{

void CrabbingPathFollower::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  pid_ = std::make_shared<project11::PID>(node, plugin_name_+".pid");

  node->declare_parameter(plugin_name_ + ".default_speed", 1.0);
  desired_speed_ = node->get_parameter(plugin_name_ + ".default_speed").as_double();

  
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  crab_angle_publisher_ = node->create_publisher<std_msgs::msg::Float32>("crab_angle", 1);
}

void CrabbingPathFollower::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller plugin %s", plugin_name_.c_str());
}


void CrabbingPathFollower::activate()
{
  RCLCPP_INFO(logger_, "Activating controller plugin %s", plugin_name_.c_str());
}

void CrabbingPathFollower::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller plugin %s", plugin_name_.c_str());
}

void CrabbingPathFollower::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (percentage)
  {
    speed_limit_percentage_ = speed_limit;
  }
  else
  {
    speed_limit_ = speed_limit;
  }
}

void CrabbingPathFollower::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
  current_segment_ = 0;
}


geometry_msgs::msg::TwistStamped CrabbingPathFollower::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{

  int segment_count = std::max<int>(0,global_plan_.poses.size()-1);

  RCLCPP_DEBUG_STREAM(logger_, "CrabbingPathFollower: segment_count: " << segment_count << " current_segment_: " << current_segment_ << " pose: " << to_yaml(pose));

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if(current_segment_ < 0 || current_segment_ > segment_count)
  {
    return cmd_vel;
  }

  // We're done if we are at the segment past the last one
  if(current_segment_ == segment_count)
    return cmd_vel;

  double target_speed = desired_speed_;
  if (speed_limit_ > 0.0)
  {
    target_speed = std::min(target_speed, speed_limit_);
  }
  else if (speed_limit_percentage_ > 0.0)
  {
    target_speed = std::min(target_speed, desired_speed_ * speed_limit_percentage_ / 100.0);
  }

  geometry_msgs::msg::PoseStamped pose_in_plan;
  try
  {
    tf_->transform(pose, pose_in_plan, global_plan_.header.frame_id);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_STREAM(logger_, "CrabbingPathFollower:  Error getting pose to plan transform: " << ex.what());
    return cmd_vel;
  }

  double segment_distance = 0.0;
  double vehicle_distance = 0.0;
  double sin_error_azimuth = 0.0;
  double cos_error_azimuth = 0.0;
  double progress = 0.0;
  geometry_msgs::msg::PoseStamped p1;
  geometry_msgs::msg::PoseStamped p2;
  project11::AngleRadians segment_azimuth;

  bool current_segment_is_good = false;

  while(!current_segment_is_good)
  {

    p1 = global_plan_.poses[current_segment_];
    p2 = global_plan_.poses[current_segment_+1];

    auto segment_dx = p2.pose.position.x - p1.pose.position.x;
    auto segment_dy = p2.pose.position.y - p1.pose.position.y;

    segment_azimuth = project11::AngleRadians(atan2(segment_dy, segment_dx));
    segment_distance = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

    // vehicle distance and azimuth relative to the segment's start point
    double dx = p1.pose.position.x - pose_in_plan.pose.position.x;
    double dy = p1.pose.position.y - pose_in_plan.pose.position.y;
    vehicle_distance = sqrt(dx*dx+dy*dy);

    project11::AngleRadians vehicle_azimuth(atan2(-dy, -dx));

    auto error_azimuth = vehicle_azimuth - segment_azimuth;
      
    sin_error_azimuth = sin(error_azimuth);
    cos_error_azimuth = cos(error_azimuth);

    // Distance traveled along the line.
    progress = vehicle_distance*cos_error_azimuth;

    if (progress < segment_distance)
      current_segment_is_good = true;
    else
    {
      current_segment_++;
      if(current_segment_ > segment_count)
      {
        return cmd_vel;
      }
    }
  }

  auto cross_track_error = vehicle_distance*sin_error_azimuth;
  auto crab_angle = project11::AngleDegrees(pid_->update(cross_track_error, pose.header.stamp));
  project11::AngleRadians heading(tf2::getYaw(pose_in_plan.pose.orientation));

  RCLCPP_DEBUG_STREAM(logger_, "CrabbingPathFollower: progress: " << progress << " cross_track_error: " << cross_track_error << " crab_angle: " << crab_angle.value() << " heading: " << heading.value() << " segment_azimuth: " << segment_azimuth.value());

  std_msgs::msg::Float32 crab_angle_msg;
  crab_angle_msg.data = crab_angle.value();
  crab_angle_publisher_->publish(crab_angle_msg);

  project11::AngleRadians target_heading = segment_azimuth +	crab_angle;

  cmd_vel.twist.angular.z = project11::AngleRadiansZeroCentered(target_heading-heading).value();

  rclcpp::Time segment_start_time = p1.header.stamp;
  rclcpp::Time segment_end_time = p2.header.stamp;
  if(segment_start_time.nanoseconds() != 0 && segment_end_time.nanoseconds() != 0 && segment_end_time > segment_start_time)
  {
    auto dt = segment_end_time - segment_start_time;
    target_speed = segment_distance/dt.seconds();
  }

  double cos_crab = std::max(cos(crab_angle), 0.5);
  cmd_vel.twist.linear.x = target_speed/cos_crab;

  return cmd_vel;

}

} // namespace controllers
} // namespace project11_navigation

PLUGINLIB_EXPORT_CLASS(project11_navigation::controllers::CrabbingPathFollower, nav2_core::Controller)

