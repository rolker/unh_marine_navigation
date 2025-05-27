#include "project11_navigation/plugins/behaviors/hover.h"
#include "nav2_util/node_utils.hpp"

namespace project11_navigation
{
Hover::Hover()
: TimedBehavior<HoverAction>()
{
}

void Hover::onConfigure()
{
  auto node = node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_+".minimum_radius", rclcpp::ParameterValue(minimum_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_+".maximum_radius", rclcpp::ParameterValue(maximum_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_+".minimum_speed", rclcpp::ParameterValue(minimum_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_+".maximum_speed", rclcpp::ParameterValue(maximum_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_+".maximum_rotation_speed", rclcpp::ParameterValue(maximum_rotation_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_+".deceleration", rclcpp::ParameterValue(deceleration_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_+".generate_visualization", rclcpp::ParameterValue(generate_visualization_));
}

nav2_behaviors::ResultStatus Hover::onRun(const std::shared_ptr<const HoverAction::Goal> goal)
{
  auto node = node_.lock();
  node->get_parameter(behavior_name_+".minimum_radius", minimum_radius_);
  node->get_parameter(behavior_name_+".maximum_radius", maximum_radius_);
  node->get_parameter(behavior_name_+".minimum_speed", minimum_speed_);
  node->get_parameter(behavior_name_+".maximum_speed", maximum_speed_);
  node->get_parameter(behavior_name_+".maximum_rotation_speed", maximum_rotation_speed_);
  node->get_parameter(behavior_name_+".deceleration", deceleration_);
  node->get_parameter(behavior_name_+".generate_visualization", generate_visualization_);

  if(generate_visualization_)
  {
    visualization_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(behavior_name_+"_visualization", 1);
    visualization_publisher_->on_activate();
  }

  if(goal->maximum_radius > 0.0)
  {
    maximum_radius_ = goal->maximum_radius;
  }
  if(goal->minimum_radius > 0.0)
  {
    minimum_radius_ = goal->minimum_radius;
  }
  if(goal->maximum_speed > 0.0)
  {
    maximum_speed_ = goal->maximum_speed;
  }

  if (!nav2_util::getCurrentPose(
    target_pose_, *this->tf_, this->local_frame_, this->robot_base_frame_,
    this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, HoverAction::Result::TF_ERROR};
  }

  return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED};
}

nav2_behaviors::ResultStatus Hover::onCycleUpdate()
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED, HoverAction::Result::TF_ERROR};
  }

  double diff_x = target_pose_.pose.position.x - current_pose.pose.position.x;
  double diff_y = target_pose_.pose.position.y - current_pose.pose.position.y;
  double current_range = hypot(diff_x, diff_y);
  double current_bearing = atan2(diff_y, diff_x);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  auto steering_angle = current_bearing - pose2d.theta;
  if (steering_angle > M_PI)
    steering_angle -= 2.0 * M_PI;
  else if (steering_angle < -M_PI)
    steering_angle += 2.0 * M_PI;

  double steering_speed = (steering_angle/M_PI) * maximum_rotation_speed_;

  double steering_proportion = abs(steering_angle) / M_PI;

  double current_target_speed = 0.0;

  if (current_range >= maximum_radius_)
  {
    current_target_speed = maximum_speed_;
  }
  else if (current_range > minimum_radius_)
  {
    float p = (current_range-minimum_radius_)/(maximum_radius_-minimum_radius_);
    current_target_speed = p*maximum_speed_;
  }
  else
  {
    if (current_range < minimum_radius_/2.0)
    {
      current_target_speed = 0.0;
      float p = 0.1*(1.0-(current_range/minimum_radius_));
      current_target_speed = -p*maximum_speed_; // apply some reverse, up to 10%
    }
    else
    {
      current_target_speed = 0.0;
    }
    // current_target_speed = 0.0;
    // steering_speed = 0.0;
  }

  if (steering_proportion > 0.25)
  {
    current_target_speed = 0.0;
  }

  current_target_speed *= (1.0 - steering_proportion*4.0);

  if (current_range > minimum_radius_)
  {
    current_target_speed = std::max(current_target_speed, minimum_speed_);
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = this->clock_->now();
  cmd_vel->header.frame_id = this->robot_base_frame_;
  cmd_vel->twist.angular.z = steering_speed;
  cmd_vel->twist.linear.x = current_target_speed;

  RCLCPP_DEBUG_STREAM(logger_, "Hover: " << diff_x << ","  << diff_y << " range: " << current_range << " angle: " << steering_angle << "\tOutput cmd_vel: " << cmd_vel->twist.linear.x << " " << cmd_vel->twist.angular.z);

  // todo - collision avoidance
  vel_pub_->publish(std::move(cmd_vel));

  if(generate_visualization_)
  {
    publish_visualization(current_pose.header.stamp);
  }

  return nav2_behaviors::ResultStatus{nav2_behaviors::Status::RUNNING};
}

void Hover::publish_visualization(rclcpp::Time time)
{
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  const auto& target = target_pose_;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = target.header.frame_id;
  // go back a few milliseconds to try to avoid tf issues
  marker.header.stamp = time-rclcpp::Duration::from_seconds(0.5);
  marker.id = 0;
  marker.ns = behavior_name_;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.pose.position =  target.pose.position;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = .75;
  marker.scale.x =  2.0*maximum_radius_;
  marker.scale.y = 2.0*maximum_radius_;
  marker.scale.z = 0.01;
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);
  marker_array->markers.push_back(marker);
  visualization_publisher_->publish(marker);

  visualization_msgs::msg::Marker inmarker;
  inmarker.header.frame_id = target.header.frame_id;
  inmarker.header.stamp = marker.header.stamp;
  inmarker.id = 1;
  inmarker.ns = behavior_name_;
  inmarker.action = visualization_msgs::msg::Marker::ADD;
  inmarker.type = visualization_msgs::msg::Marker::SPHERE;
  inmarker.pose.position =  target.pose.position;
  inmarker.pose.orientation.w = 1.0;
  inmarker.color.r = 0.0;
  inmarker.color.g = 1.0;
  inmarker.color.b = 0.2;
  inmarker.color.a = .75;
  inmarker.scale.x =  2.0*minimum_radius_;
  inmarker.scale.y = 2.0*minimum_radius_;
  inmarker.scale.z = 0.01;
  inmarker.lifetime = rclcpp::Duration::from_seconds(2.5);
  marker_array->markers.push_back(inmarker);
  visualization_publisher_->publish(inmarker);

  //visualization_publisher_->publish(std::move(marker_array));
}


} // namespace project11_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(project11_navigation::Hover, nav2_core::Behavior)
