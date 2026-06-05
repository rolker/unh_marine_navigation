#ifndef MARINE_NAV_CRABBING_PATH_FOLLOWER_CRABBING_PATH_FOLLOWER_H
#define MARINE_NAV_CRABBING_PATH_FOLLOWER_CRABBING_PATH_FOLLOWER_H

#include <atomic>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
//#include "project11/pid.h"
#include "control_toolbox/pid_ros.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace marine_nav_crabbing_path_follower
{

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

  bool visualize_ = false;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;

  // Live updates to `default_speed` via the node's parameter service —
  // without this `desired_speed_` is captured once at configure() and
  // never updated.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
};


} // namespace marine_nav_crabbing_path_follower

#endif