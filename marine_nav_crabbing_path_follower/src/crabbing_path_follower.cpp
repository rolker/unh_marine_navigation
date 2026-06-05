#include "marine_nav_crabbing_path_follower/crabbing_path_follower.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

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
    // Accept both PARAMETER_DOUBLE and PARAMETER_INTEGER. YAML `default_speed: 2`
    // (no decimal) parses as integer — a common operator mistake worth
    // coercing rather than silently rejecting. Other types route through
    // the invalid branch below.
    double initial_value;
    bool type_ok = true;
    const auto initial_type = initial_param.get_type();
    if (initial_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      initial_value = initial_param.as_double();
    } else if (initial_type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      initial_value = static_cast<double>(initial_param.as_int());
    } else {
      initial_value = std::numeric_limits<double>::quiet_NaN();
      type_ok = false;
    }
    if (type_ok && std::isfinite(initial_value) && initial_value > 0.0) {
      desired_speed_.store(initial_value);
    } else {
      constexpr double kFallback = 1.0;
      // Use value_to_string() so the WARN shows the actual provided value
      // (e.g. "false", "[1, 2]") rather than the nan placeholder we use
      // internally to route wrong-type cases through the invalid branch.
      RCLCPP_WARN(
        logger_,
        "CrabbingPathFollower: configured default_speed is invalid "
        "(type=%s, value=%s; must be a finite positive number); "
        "falling back to %.3f m/s",
        rclcpp::to_string(initial_type).c_str(),
        initial_param.value_to_string().c_str(), kFallback);
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

      // Coerce a numeric (double or integer) parameter; sets result.reason and
      // returns false for a non-numeric type.
      auto as_number = [&result](const rclcpp::Parameter & p, double & out) -> bool {
        const auto t = p.get_type();
        if (t == rclcpp::ParameterType::PARAMETER_DOUBLE) {out = p.as_double(); return true;}
        if (t == rclcpp::ParameterType::PARAMETER_INTEGER) {
          out = static_cast<double>(p.as_int());
          return true;
        }
        result.successful = false;
        result.reason = p.get_name() +
          " must be numeric (double or integer); got '" + p.value_to_string() + "'";
        return false;
      };
      // Validate (finite + lower bound) and live-update one tuning atomic.
      auto update = [this, &result](
          std::atomic<double> & target, double v, double lo, bool exclusive_lo,
          const char * pn) -> bool {
        if (!(std::isfinite(v) && (exclusive_lo ? v > lo : v >= lo))) {
          result.successful = false;
          result.reason = std::string(pn) + " must be finite and " +
            (exclusive_lo ? "> " : ">= ") + std::to_string(lo) + "; got " + std::to_string(v);
          return false;
        }
        const double prev = target.load();
        if (v != prev) {
          target.store(v);
          RCLCPP_INFO(logger_, "CrabbingPathFollower: %s updated %.3f -> %.3f", pn, prev, v);
        }
        return true;
      };
      const std::string base = plugin_name_;
      for (const auto & p : params) {
        const std::string & name = p.get_name();
        if (name == default_speed_name) {
        // Accept PARAMETER_DOUBLE and PARAMETER_INTEGER (CLI `ros2 param set
        // ... 2` parses as integer; common operator mistake worth coercing).
        // Reject other types explicitly so callers see the error instead of
        // a silent state mismatch between param service and controller.
        // Mirrors the configure-time type check above.
        double new_value;
        const auto p_type = p.get_type();
        if (p_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          new_value = p.as_double();
        } else if (p_type == rclcpp::ParameterType::PARAMETER_INTEGER) {
          new_value = static_cast<double>(p.as_int());
        } else {
          result.successful = false;
          result.reason =
            "default_speed must be a numeric type (double or integer); got type='" +
            rclcpp::to_string(p_type) + "', value='" +
            p.value_to_string() + "'";
          return result;
        }
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
        } else if (name == base + ".heading_rate_gain") {
          double v;
          if (!as_number(p, v) || !update(heading_rate_gain_, v, 0.0, true, "heading_rate_gain")) {
            return result;
          }
        } else if (name == base + ".max_yaw_rate") {
          double v;
          if (!as_number(p, v) || !update(max_yaw_rate_, v, 0.0, true, "max_yaw_rate")) {
            return result;
          }
        } else if (name == base + ".lookahead_distance") {
          double v;
          if (!as_number(p, v) ||
            !update(lookahead_distance_, v, 0.0, false, "lookahead_distance"))
          {
            return result;
          }
        } else if (name == base + ".lookahead_time") {
          double v;
          if (!as_number(p, v) || !update(lookahead_time_, v, 0.0, false, "lookahead_time")) {
            return result;
          }
        } else if (name == base + ".lookahead_min_distance") {
          double v;
          if (!as_number(p, v) ||
            !update(lookahead_min_distance_, v, 0.0, false, "lookahead_min_distance"))
          {
            return result;
          }
        } else if (name == base + ".new_plan_goal_tolerance") {
          double v;
          if (!as_number(p, v) ||
            !update(new_plan_goal_tolerance_, v, 0.0, false, "new_plan_goal_tolerance"))
          {
            return result;
          }
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

  // Inner heading loop + pure-pursuit look-ahead (see header). All defaults
  // reproduce the historical segment-azimuth behaviour, so nothing changes
  // until tuned. Declared + validated here, kept live-tunable by the parameter
  // callback below (same finite/lower-bound validation). exclusive_lo gates
  // ">lo" (gains, max_yaw_rate) vs ">=lo" (look-ahead distances / tolerance).
  auto read_validated = [&](const std::string & suffix, std::atomic<double> & target,
      double lo, bool exclusive_lo) {
      double v = target.load();
      const std::string pname = plugin_name_ + suffix;
      nav2_util::declare_parameter_if_not_declared(node, pname, rclcpp::ParameterValue(v));
      node->get_parameter(pname, v);
      if (std::isfinite(v) && (exclusive_lo ? v > lo : v >= lo)) {
        target.store(v);
      } else {
        RCLCPP_WARN(
          logger_, "CrabbingPathFollower: %s=%.3f invalid; keeping %.3f",
          pname.c_str(), v, target.load());
      }
    };
  read_validated(".heading_rate_gain", heading_rate_gain_, 0.0, true);
  read_validated(".max_yaw_rate", max_yaw_rate_, 0.0, true);
  read_validated(".lookahead_distance", lookahead_distance_, 0.0, false);
  read_validated(".lookahead_time", lookahead_time_, 0.0, false);
  read_validated(".lookahead_min_distance", lookahead_min_distance_, 0.0, false);
  read_validated(".new_plan_goal_tolerance", new_plan_goal_tolerance_, 0.0, false);

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

  // Progress-preserving localization. The avoidance decorator
  // (marine_nav_avoidance_controller, #59) re-issues setPlan every control
  // cycle with a freshly reshaped — but same-goal — path; a same-goal replan
  // (IsPathValid failure) does likewise. Resetting current_segment_ to 0 each
  // time made the forward scan in computeVelocityCommands re-localise from the
  // path start, which during a weave/loop snaps the cursor *backward* and steps
  // the cross-track reference 5-9 m (kicking the PID — root-caused 2026-06-04).
  // Keep the cursor across same-goal re-issues; reset only when the goal (final
  // pose) moves more than new_plan_goal_tolerance_ — a genuinely new line — so
  // each new line still starts from its beginning. The PID is likewise not
  // reset here (only by the staleness guard in computeVelocityCommands).
  const auto & poses = path.poses;
  bool new_line = true;
  if (!poses.empty() && have_last_goal_) {
    const auto & g = poses.back().pose.position;
    const double moved = std::hypot(g.x - last_goal_.x, g.y - last_goal_.y);
    new_line = moved > new_plan_goal_tolerance_.load();
  }
  if (new_line || current_segment_ < 0) {
    current_segment_ = 0;
  }
  const int segment_count = std::max<int>(0, static_cast<int>(poses.size()) - 1);
  if (current_segment_ > segment_count) {
    // A sparser/shorter same-goal re-plan left the cursor past the new path.
    // Re-localize onto the last *traversable* segment, not segment_count —
    // current_segment_ == segment_count is the "done" sentinel in
    // computeVelocityCommands, which would stall the boat (zero cmd_vel).
    current_segment_ = std::max(0, segment_count - 1);
  }
  if (!poses.empty()) {
    last_goal_ = poses.back().pose.position;
    have_last_goal_ = true;
  }

  global_plan_ = path;
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

  // Bounded backward re-localization (at most one segment per cycle). The
  // forward scan below only ever advances the cursor; if a same-goal path
  // reshape moved a vertex *ahead* of the boat, the cursor would stay too far
  // forward and measure cross-track against a segment the boat hasn't reached.
  // If the boat now projects behind the current segment's start, step back one
  // segment. The single-step bound is deliberate: it lets a reshape re-localize
  // without a weave/loop snapping the cursor all the way back to the start (the
  // regression this change fixes). current_segment_ is in [1, segment_count-1]
  // inside this guard, so indexing current_segment_+1 stays in range.
  if (current_segment_ > 0) {
    const double proj = alongTrackProjection(
      global_plan_.poses[current_segment_].pose.position,
      global_plan_.poses[current_segment_ + 1].pose.position,
      pose_in_plan.pose.position);
    if (proj < 0.0) {
      current_segment_--;
    }
  }

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

  // Base heading: the local segment azimuth (default), or — with look-ahead
  // enabled — the pure-pursuit bearing to a point `lookahead` metres ahead on
  // the path, which anticipates bends instead of reacting to them. The look-
  // ahead distance is fixed (lookahead_distance_) or speed-scaled
  // (lookahead_time_ > 0: L = max(lookahead_min_distance_, V*time)).
  // Snapshot the live-tunable atomics once so a mid-cycle `ros2 param set`
  // can't tear the control law.
  const double lookahead_distance = lookahead_distance_.load();
  const double lookahead_time = lookahead_time_.load();
  const double lookahead_min_distance = lookahead_min_distance_.load();

  AngleRadians base_heading = segment_azimuth;
  double lookahead = lookahead_distance;
  if (lookahead_time > 0.0) {
    // Speed-scaled horizon. Note target_speed here is the commanded speed
    // (default_speed / speed-limit); the per-pose-timestamp trajectory speed
    // is derived later (below), so lookahead_time scales off the commanded
    // speed, not the trajectory-derived one.
    lookahead = std::max(lookahead_min_distance, target_speed * lookahead_time);
  }
  if (lookahead > 0.0) {
    // progress is the boat's signed projection along the current segment; it is
    // clamped to [0, seg_len] inside lookaheadPoint, so a boat sitting behind
    // the segment start measures the horizon from the segment start.
    const auto la = lookaheadPoint(
      global_plan_.poses, current_segment_, progress, lookahead);
    base_heading = AngleRadians(atan2(
      la.y - pose_in_plan.pose.position.y,
      la.x - pose_in_plan.pose.position.x));
  }

  AngleRadians target_heading = base_heading + crab_angle;

  // Inner heading loop: proportional heading-error -> yaw rate, with a tunable
  // gain and a clamp to the achievable yaw envelope. Defaults (gain 1.0, clamp
  // +/-pi) reproduce the previous ZeroCentered-wrap behaviour. The
  // velocity_smoother still enforces the physical rate/accel limits downstream.
  const double heading_error =
    AngleRadiansZeroCentered(target_heading - heading).value();
  const double heading_rate_gain = heading_rate_gain_.load();
  const double max_yaw_rate = max_yaw_rate_.load();
  cmd_vel.twist.angular.z =
    std::clamp(heading_rate_gain * heading_error, -max_yaw_rate, max_yaw_rate);

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
