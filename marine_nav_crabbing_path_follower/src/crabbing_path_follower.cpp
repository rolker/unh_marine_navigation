#include "marine_nav_crabbing_path_follower/crabbing_path_follower.h"

#include <cmath>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "marine_nav_utilities/gz4d/angles.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace marine_nav_crabbing_path_follower
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

  pid_ = std::make_shared<control_toolbox::PidROS>(node, plugin_name_+".pid", plugin_name_+"/pid");
  control_toolbox::AntiWindupStrategy pid_anti_windup_strategy;
  pid_anti_windup_strategy.set_type("conditional_integration");
  pid_anti_windup_strategy.i_max = 75.0;
  pid_anti_windup_strategy.i_min = -75.0;
  pid_->initialize_from_args(1.0, 0.0, 0.0, 90.0, -90.0, pid_anti_windup_strategy, false);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".pid.reset_threshold_seconds", rclcpp::ParameterValue(1.0));
  double pid_reset_threshold_seconds = node->get_parameter(plugin_name_ + ".pid.reset_threshold_seconds").as_double();
  pid_reset_threshold_ = rclcpp::Duration::from_seconds(pid_reset_threshold_seconds);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".default_speed", rclcpp::ParameterValue(1.0));
  // Apply the same type + isfinite + >0 guard at configure-time that the
  // param callback below applies to live updates. Without this, an
  // invalid YAML/launch value (wrong type, NaN/Inf, <=0) propagates into
  // computeVelocityCommands before any SetParameters update can correct it.
  // The wrong-type case is real: `default_speed: 1` in YAML parses as
  // integer (PARAMETER_INTEGER), and a bare `as_double()` then throws
  // InvalidParameterTypeException — which would abort configure() instead
  // of falling back, defeating the safety guard.
  {
    const std::string default_speed_param = plugin_name_ + ".default_speed";
    const auto initial_param = node->get_parameter(default_speed_param);
    const bool type_ok =
      initial_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE;
    const double initial_value = type_ok
      ? initial_param.as_double()
      : std::numeric_limits<double>::quiet_NaN();
    if (type_ok && std::isfinite(initial_value) && initial_value > 0.0) {
      desired_speed_.store(initial_value);
    } else {
      constexpr double kFallback = 1.0;
      RCLCPP_WARN(
        logger_,
        "CrabbingPathFollower: configured default_speed is invalid "
        "(type=%s, value=%.3f; must be PARAMETER_DOUBLE, finite, > 0); "
        "falling back to %.3f m/s",
        rclcpp::to_string(initial_param.get_type()).c_str(),
        initial_value, kFallback);
      desired_speed_.store(kFallback);
      // Write the fallback back to the parameter so the param service reports
      // the effective value. Without this, `ros2 param get` would keep showing
      // the original invalid value while the controller runs at the fallback,
      // which is misleading at field-debug time. The on-set-parameters
      // callback isn't registered yet at this point in configure(), so this
      // doesn't re-trigger validation. Wrapped in try/catch defensively —
      // set_parameter can throw if the node is being torn down concurrently.
      try {
        node->set_parameter(rclcpp::Parameter(default_speed_param, kFallback));
      } catch (const std::exception & e) {
        // Broad catch: covers all rclcpp::exceptions::* (e.g., RCLError,
        // InvalidParameterValueException from a future param validator) plus
        // any std-derived exception. Narrower catches risk propagating out of
        // configure() and failing controller bring-up on the very edge case
        // the fallback is trying to defend against.
        RCLCPP_WARN(
          logger_,
          "CrabbingPathFollower: could not update default_speed parameter to "
          "fallback %.3f (param service refused): %s",
          kFallback, e.what());
      }
    }
  }

  // Live updates: a SetParameters call against this node (from `ros2 param set`
  // or a BT plugin per task) will update `desired_speed_` in place. Other
  // parameters are passed through unchanged. Only logs when the value
  // actually changes to avoid log spam if some caller pushes the same
  // value repeatedly.
  const std::string default_speed_name = plugin_name_ + ".default_speed";
  params_cb_handle_ = node->add_on_set_parameters_callback(
    [this, default_speed_name](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() != default_speed_name) {
          continue;
        }
        // Reject wrong-type updates explicitly. Without this, the AND-
        // condition that used to gate validation here silently fell through
        // when name matched but type was wrong — leaving result.successful
        // at its default `true`. That meant `ros2 param set
        // /<ns>/controller_server FollowPath.default_speed 1` (integer)
        // would succeed in the param service but leave `desired_speed_`
        // unchanged: observability mismatch between `ros2 param get` and
        // the controller's effective speed. Mirrors the configure-time
        // type check above.
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason =
            "default_speed must be PARAMETER_DOUBLE; got type='" +
            rclcpp::to_string(p.get_type()) + "'";
          return result;
        }
        const double new_value = p.as_double();
        // Reject non-finite or non-positive speeds at the parameter boundary.
        // Without this guard NaN/Inf propagates into desired_speed_ and then
        // into computeVelocityCommands' target_speed, commanding NaN cmd_vel
        // on an autonomous boat. The BT-side SetControllerSpeed plugin has
        // its own isfinite check; this is defense in depth (operators may
        // also `ros2 param set` directly).
        if (!std::isfinite(new_value) || new_value <= 0.0) {
          result.successful = false;
          result.reason =
            "default_speed must be a finite positive value (m/s); got " +
            std::to_string(new_value);
          // Don't update desired_speed_; the operator's request is rejected
          // and the controller stays at its prior value.
          return result;
        }
        const double prev_value = desired_speed_.load();
        if (new_value != prev_value) {
          desired_speed_.store(new_value);
          RCLCPP_INFO(
            logger_,
            "CrabbingPathFollower: default_speed updated %.3f -> %.3f m/s",
            prev_value, new_value);
        }
      }
      return result;
    });

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".visualization", rclcpp::ParameterValue(visualize_));
  visualize_ = node->get_parameter(plugin_name_ + ".visualization").as_bool();
  if(visualize_)
  {
    visualization_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "path_follower_visualization", 1);
  }

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void CrabbingPathFollower::cleanup()
{
  // Release the parameter callback registered in configure() so rclcpp's
  // callback list no longer holds a lambda capturing `this`. Pairs with
  // the registration above; without it the callback would only release
  // at destructor time, which is fine in well-ordered teardown but
  // leaves a small lifecycle window where a parameter SetParameters
  // could trigger the callback against a plugin in the wrong lifecycle
  // state. Aligns with the nav2 controller plugin idiom of mirroring
  // configure()'s resource acquisition in cleanup().
  params_cb_handle_.reset();
  RCLCPP_INFO(logger_, "Cleaning up controller plugin %s", plugin_name_.c_str());
}


void CrabbingPathFollower::activate()
{
  global_pub_->on_activate();
  if (visualize_)
  {
    visualization_publisher_->on_activate();
  }
  pid_->initialize_from_ros_parameters();

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
  pid_->reset(true);
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

  // Snapshot the atomic once per cycle so the speed-limit math and the
  // DEBUG log below see the same value, even if a SetParameters update
  // lands mid-cycle. (`desired_speed_` is `std::atomic<double>` so
  // individual `load()`s are already tear-free; this is about
  // intra-cycle consistency, not tearing.)
  const double desired_speed_snapshot = desired_speed_.load();
  double target_speed = desired_speed_snapshot;
  if (speed_limit_ > 0.0)
  {
    if(speed_limit_is_percentage_)
      target_speed = std::min(target_speed, desired_speed_snapshot * speed_limit_ / 100.0);
    else
      target_speed = std::min(target_speed, speed_limit_);
  }

  RCLCPP_DEBUG_STREAM(logger_, "CrabbingPathFollower: target_speed: " << target_speed << " desired_speed_: " << desired_speed_snapshot << " speed_limit_: " << speed_limit_ << " speed_limit_is_percentage_: " << speed_limit_is_percentage_);

  geometry_msgs::msg::PoseStamped pose_in_plan;
  nav2_util::transformPoseInTargetFrame(
    pose, pose_in_plan, *tf_,
    costmap_ros_->getGlobalFrameID(),
    transform_tolerance_);

  using marine_nav_utilities::gz4d::AngleDegrees;
  using marine_nav_utilities::gz4d::AngleRadians;
  using marine_nav_utilities::gz4d::AngleRadiansZeroCentered;

  double segment_distance = 0.0;
  double vehicle_distance = 0.0;
  double sin_error_azimuth = 0.0;
  double cos_error_azimuth = 0.0;
  double progress = 0.0;
  geometry_msgs::msg::PoseStamped p1;
  geometry_msgs::msg::PoseStamped p2;
  AngleRadians segment_azimuth;

  bool current_segment_is_good = false;

  while(!current_segment_is_good)
  {

    p1 = global_plan_.poses[current_segment_];
    p2 = global_plan_.poses[current_segment_+1];

    auto segment_dx = p2.pose.position.x - p1.pose.position.x;
    auto segment_dy = p2.pose.position.y - p1.pose.position.y;

    segment_azimuth = AngleRadians(atan2(segment_dy, segment_dx));
    segment_distance = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

    // vehicle distance and azimuth relative to the segment's start point
    double dx = p1.pose.position.x - pose_in_plan.pose.position.x;
    double dy = p1.pose.position.y - pose_in_plan.pose.position.y;
    vehicle_distance = sqrt(dx*dx+dy*dy);

    AngleRadians vehicle_azimuth(atan2(-dy, -dx));

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

  rclcpp::Time timestamp(pose.header.stamp);

  if(last_update_time_.nanoseconds() == 0 ||  (timestamp - last_update_time_) > pid_reset_threshold_ || (timestamp - last_update_time_).seconds() < 0.0)
  {
    pid_->reset();
    last_update_time_ = timestamp;
  }
  auto dt = timestamp - last_update_time_;
  last_update_time_ = timestamp;

  auto cross_track_error = vehicle_distance*sin_error_azimuth;
  auto crab_angle = AngleDegrees(pid_->compute_command(cross_track_error, dt));
  AngleRadians heading(tf2::getYaw(pose_in_plan.pose.orientation));

  RCLCPP_DEBUG_STREAM(logger_, "CrabbingPathFollower: progress: " << progress << " cross_track_error: " << cross_track_error << " crab_angle: " << crab_angle.value() << " heading: " << heading.value() << " segment_azimuth: " << segment_azimuth.value());

  AngleRadians target_heading = segment_azimuth +	crab_angle;

  cmd_vel.twist.angular.z = AngleRadiansZeroCentered(target_heading-heading).value();

  rclcpp::Time segment_start_time = p1.header.stamp;
  rclcpp::Time segment_end_time = p2.header.stamp;
  if(segment_start_time.nanoseconds() != 0 && segment_end_time.nanoseconds() != 0 && segment_end_time > segment_start_time)
  {
    auto dt = segment_end_time - segment_start_time;
    target_speed = segment_distance/dt.seconds();
  }

  double cos_crab = std::max(cos(crab_angle), 0.5);
  cmd_vel.twist.linear.x = target_speed/cos_crab;

  RCLCPP_DEBUG_STREAM(logger_, "CrabbingPathFollower: target_speed (after potential trajectory derivation): " << target_speed << " adjusted for crab angle: " << cmd_vel.twist.linear.x);

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
  colors[1].r = 0.25;
  colors[1].g = 0.75;
  colors[1].b = 0.25;
  colors[1].a = 0.75;
  // future_color
  colors[2].r = 0.25;
  colors[2].g = 0.25;
  colors[2].b = 0.75;
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

} // namespace marine_nav_crabbing_path_follower

PLUGINLIB_EXPORT_CLASS(marine_nav_crabbing_path_follower::CrabbingPathFollower, nav2_core::Controller)
