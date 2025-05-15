#include "project11_navigation/plugins/controllers/crabbing_path_follower.h"
#include "project11/utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

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

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".default_speed", rclcpp::ParameterValue(1.0));
  desired_speed_ = node->get_parameter(plugin_name_ + ".default_speed").as_double();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".visualization", rclcpp::ParameterValue(visualize_));
  visualize_ = node->get_parameter(plugin_name_ + ".visualization").as_bool();
  if(visualize_)
  {
    visualization_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "path_follower_visualization", 1);
  }
  
  double transform_tolerance;
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(transform_tolerance_.seconds()));
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void CrabbingPathFollower::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller plugin %s", plugin_name_.c_str());
}


void CrabbingPathFollower::activate()
{
  global_pub_->on_activate();
  if (visualize_)
  {
    visualization_publisher_->on_activate();
  }
  
  RCLCPP_INFO(logger_, "Activating controller plugin %s", plugin_name_.c_str());
}

void CrabbingPathFollower::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller plugin %s", plugin_name_.c_str());
}

void CrabbingPathFollower::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
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
    if(speed_limit_is_percentage_)
      target_speed = std::min(target_speed, desired_speed_ * speed_limit_ / 100.0);
    else
      target_speed = std::min(target_speed, speed_limit_);
  }

  RCLCPP_INFO_STREAM(logger_, "CrabbingPathFollower: target_speed: " << target_speed << " desired_speed_: " << desired_speed_ << " speed_limit_: " << speed_limit_ << " speed_limit_is_percentage_: " << speed_limit_is_percentage_);

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

  RCLCPP_INFO_STREAM(logger_, "CrabbingPathFollower: target_speed (after potential trajectory derivation): " << target_speed << " adjusted for crab angle: " << cmd_vel.twist.linear.x);

  if(visualize_)
  {
    publish_visualization(cmd_vel);
  }

  return cmd_vel;

}

void CrabbingPathFollower::publish_visualization(
  const geometry_msgs::msg::TwistStamped & cmd_vel)
{
  if(!visualize_)
    return;

  visualization_msgs::msg::MarkerArray marker_array;
  
  std::array<std_msgs::msg::ColorRGBA, 3> colors;
  // past_color
  colors[0].r = 0.25;
  colors[0].g = 0.25;
  colors[0].b = 0.25;
  colors[0].a = 0.5;
  // current_color
  colors[1].r = 0.35;
  colors[1].g = 0.35;
  colors[1].b = 0.5;
  colors[1].a = 0.75;
  // future_color
  colors[2].r = 0.25;
  colors[2].g = 0.25;
  colors[2].b = 0.4;
  colors[2].a = 0.5;

  if(!global_plan_.poses.empty())
  {
    const auto& poses = global_plan_.poses;
    std::vector<visualization_msgs::msg::Marker> markers(3);
    for(int i = 0; i < markers.size(); i++)
    {
      markers[i].header.frame_id = poses.front().header.frame_id;
      markers[i].header.stamp = cmd_vel.header.stamp;
      markers[i].id = i;
      markers[i].ns = plugin_name_;
      markers[i].action = visualization_msgs::msg::Marker::ADD;
      markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
      markers[i].pose.orientation.w = 1.0;
      markers[i].color = colors[i];
      markers[i].scale.x = 1.0;
      markers[i].lifetime = rclcpp::Duration::from_seconds(2.0);
    }

    int markers_index = 0; // start with past
    for(int i = 0; i < poses.size()-1; i++)
    {
      // still working on past markers?
      if(markers_index == 0)
      {
        // did we reach the current segment?
        if(i == current_segment_)
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
      marker_array.markers.push_back(marker);
  }


  visualization_publisher_->publish(marker_array);
}

} // namespace controllers
} // namespace project11_navigation

PLUGINLIB_EXPORT_CLASS(project11_navigation::controllers::CrabbingPathFollower, nav2_core::Controller)

