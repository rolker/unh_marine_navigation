#ifndef MARINE_NAV_CRABBING_PATH_FOLLOWER_CRABBING_PATH_FOLLOWER_H
#define MARINE_NAV_CRABBING_PATH_FOLLOWER_CRABBING_PATH_FOLLOWER_H

#include <atomic>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
//#include "project11/pid.h"
#include "control_toolbox/pid_ros.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "marine_control/control_server.hpp"

namespace marine_nav_crabbing_path_follower
{

// Declare `<name>.default_speed` (the commanded survey speed) on `node` with a
// FloatingPointRange descriptor so the marine_control panel renders it as a
// bounded float. Kept separate from declareCrabbingControlParams below because
// default_speed already has bespoke configure-time validation + graceful
// fallback (invalid YAML/launch values fall back rather than aborting bring-up);
// the range is deliberately permissive ([0, 20] m/s — wider than any realistic
// survey-USV speed) so that fallback is preserved for every physically
// meaningful value and only a self-contradictory out-of-range override fails
// loudly at declare (matching the avoidance sibling's stance). Free function so
// the gtest can declare default_speed identically without a full configure().
void declareCrabbingDefaultSpeed(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);

// Declare the live operator-tunable parameters on `node` under `<name>.*`, each
// with a FloatingPointRange descriptor whose bounds come from a startup-only
// companion parameter `<name>.<tunable>_range` ([min, max] array) so platforms
// can customise them. A malformed range falls back to a built-in default
// (warned). Mirrors marine_nav_avoidance_controller::declareAvoidanceControlParams.
// Excludes default_speed (handled by declareCrabbingDefaultSpeed). Free function
// so the gtest can exercise the real declaration logic without a full nav2
// controller_server bring-up.
void declareCrabbingControlParams(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);

// Bind the live crabbing tunables (declared by declareCrabbingDefaultSpeed +
// declareCrabbingControlParams) to `server` as marine_control controls, with
// their units and UI groups. Binds default_speed plus the eleven tunables.
void bindCrabbingControls(marine_control::ControlServer & server, const std::string & name);

class CrabbingPathFollower : public nav2_core::Controller
{
public:
  CrabbingPathFollower() = default;
  ~CrabbingPathFollower() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  void publish_visualization(const geometry_msgs::msg::TwistStamped & cmd_vel);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("CrabbingPathFollower")};
  rclcpp::Clock::SharedPtr clock_;

  // Atomic because the parameter-service callback (rclcpp services thread)
  // writes this while computeVelocityCommands (controller-server compute
  // thread) reads it. Plain `double` would be UB per the C++ memory model.
  std::atomic<double> desired_speed_{0.0};
  double speed_limit_ = -1.0;
  bool speed_limit_is_percentage_ = false;


  double transform_tolerance_ {0.0};

  int current_segment_ = -1;
  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

  std::shared_ptr<control_toolbox::PidROS> pid_;
  rclcpp::Time last_update_time_;
  rclcpp::Duration pid_reset_threshold_ {1, 0};

  // Inner heading loop: angular.z = clamp(heading_rate_gain_ * heading_error,
  // ±max_yaw_rate_). Defaults reproduce the historical behaviour (gain 1.0,
  // clamp at ±pi == the previous ZeroCentered wrap). The velocity_smoother
  // still enforces the physical rate/accel limits downstream — this clamp is a
  // controller-level safety bound, not a replacement.
  //
  // These are live-tunable via `ros2 param set` (the parameter callback below
  // validates + updates them), so they are `std::atomic` for the same reason as
  // desired_speed_: the param-service thread writes while computeVelocityCommands
  // reads on the controller thread.
  std::atomic<double> heading_rate_gain_{1.0};
  std::atomic<double> max_yaw_rate_{M_PI};

  // Pure-pursuit look-ahead. When the effective distance is > 0 the base
  // heading aims at a point that far ahead on the path (anticipates bends)
  // instead of the local segment azimuth. lookahead_time_ > 0 makes the
  // distance speed-scaled: L = max(lookahead_min_distance_, V * time).
  // Look-ahead is OFF by default: lookahead_distance_ and lookahead_time_ are
  // both 0, so the base heading stays the segment azimuth (historical
  // behaviour) until tuned. lookahead_min_distance_ (1.0) is only a floor that
  // applies once lookahead_time_ > 0. With look-ahead on, set the cross-track
  // PID to I-only (the look-ahead distance becomes the cross-track-tightness
  // dial; I keeps the current crab).
  std::atomic<double> lookahead_distance_{0.0};
  std::atomic<double> lookahead_time_{0.0};
  std::atomic<double> lookahead_min_distance_{1.0};

  // Progress-preserving localization: keep the segment cursor across the
  // per-cycle setPlan re-issues from the avoidance decorator, resetting only
  // when the goal (final pose) moves more than this — i.e. a genuinely new
  // line. Stops the cross-track reference snapping backward during a weave.
  // have_last_goal_/last_goal_ are touched only by setPlan (controller thread),
  // so they stay plain; the tolerance is live-tunable, hence atomic.
  bool have_last_goal_ = false;
  geometry_msgs::msg::Point last_goal_;
  std::atomic<double> new_plan_goal_tolerance_{1.0};

  // Cross-track-error slew limiter (#66). Caps how fast the cross-track error
  // fed to the PID may change (m/s) so a discontinuous reference step — a
  // planner replan or the avoidance decorator reshaping the line under the boat
  // — is ramped in rather than slamming the controller into an over-correction
  // (the hunting / 360-loop family). Genuine lateral drift changes at most at
  // boat speed, well under a sane rate, so real tracking is untouched. 0.0
  // disables (the default; historical behaviour until tuned). The rate is
  // live-tunable, hence atomic; slewed_cross_track_error_ / slew_initialized_
  // are control-loop state touched only in computeVelocityCommands, so plain.
  std::atomic<double> cross_track_error_slew_rate_{0.0};
  double slewed_cross_track_error_{0.0};
  bool slew_initialized_{false};

  // Cross-track gain schedule (speed-normalization, #76). The outer cross-track
  // loop's effective gain is proportional to commanded speed v (ė ≈ v·sin(crab)),
  // so fixed PID gains lose stability margin as speed rises — a speed-dependent
  // cross-track limit cycle (Lake Massabesic, unh_echoboats_project11#289).
  // Scaling the PID output crab_angle by gain_ref_speed / max(v, v_min) cancels
  // that plant gain v, holding the closed-loop response constant across speed.
  // pid_gain_ref_speed_ = 0.0 DISABLES it (the default; no behavior change until a
  // platform opts in, mirroring lookahead_time_ = 0). pid_gain_v_min_ floors the
  // divisor so creep / station-keep (v → 0) can't blow the gain up or divide by
  // zero; its validator enforces > 0 (a floor of 0 would re-admit the blow-up).
  // Both are live-tunable via the parameter callback, hence atomic (param-service
  // thread writes while computeVelocityCommands reads on the controller thread).
  std::atomic<double> pid_gain_ref_speed_{0.0};
  std::atomic<double> pid_gain_v_min_{0.5};

  // Turn-speed regulation (#87). computeVelocityCommands commands
  // linear.x = target_speed / cos(crab_angle); when the cross-track PID drives a
  // large crab angle on a turn, that division inflates the surge exactly when the
  // boat is turning hardest (+18% commanded surge, +60% modelled current draw in
  // turns vs. straights; field data 2026-06-30). turnSpeedFactor() pre-multiplies
  // target_speed by clamp(1 - |crab_angle| / turn_speed_max_crab_deg_,
  // turn_speed_min_factor_, 1.0) to counter that inflation, slowing the boat in
  // proportion to how hard it is crabbing.
  // turn_speed_max_crab_deg_ = 0.0 DISABLES it (the default; no behavior change
  // until a platform opts in, mirroring pid_gain_ref_speed_ = 0). When enabled,
  // turn_speed_min_factor_ (default 0.3) floors the factor so the boat never
  // stalls mid-turn — 30% of target_speed at maximum regulation. Both are
  // live-tunable via the parameter callback, hence atomic (param-service thread
  // writes while computeVelocityCommands reads on the controller thread).
  std::atomic<double> turn_speed_max_crab_deg_{0.0};
  std::atomic<double> turn_speed_min_factor_{0.3};

  bool visualize_ = false;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;

  // Live updates to `default_speed` via the node's parameter service —
  // without this `desired_speed_` is captured once at configure() and
  // never updated.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  // marine_control panel namespace for this controller's control topics
  // (~/control/<namespace>/state|change), read from
  // `<plugin_name_>.marine_control.namespace` in configure(). Defaults to
  // plugin_name_, which reproduces the standalone topic layout byte-for-byte.
  // When this follower is wrapped under the SAME plugin name (e.g. by
  // marine_nav_avoidance_controller, which configures + activates its inner
  // controller as plugin_name_), the deployment sets this to a distinct value so
  // the inner ControlServer and the wrapper's ControlServer don't both advertise
  // on identical topics (two state publishers -> an incoherent panel set).
  std::string marine_control_namespace_;

  // Boat-side marine_control server, attached to the parent controller_server
  // node. Publishes the live tunables as bridgeable controls and applies
  // operator changes via the validated parameter path (the same atomics the
  // on-set-parameters callback writes, so the marine_control `change` path and
  // `ros2 param set` stay consistent). Lives only while the controller is active
  // (created in activate, reset in deactivate/cleanup).
  std::shared_ptr<marine_control::ControlServer> control_server_;
};


} // namespace marine_nav_crabbing_path_follower

#endif