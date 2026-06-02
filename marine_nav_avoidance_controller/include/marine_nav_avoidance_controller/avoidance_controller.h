#ifndef MARINE_NAV_AVOIDANCE_CONTROLLER_AVOIDANCE_CONTROLLER_H
#define MARINE_NAV_AVOIDANCE_CONTROLLER_AVOIDANCE_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "marine_nav_utilities/corridor_solver.h"

namespace marine_nav_avoidance_controller
{

// A decorator nav2_core::Controller that reshapes the followed path around
// obstacles before delegating to an inner ("primary") controller. The inner
// controller stays a pure path tracker (e.g. CrabbingPathFollower); this wrapper
// owns the obstacle-avoidance corridor solve. Unlike the BT-node avoider (#30),
// it reads the controller_server's own in-process costmap
// (costmap_ros_->getCostmap()) — fresh by construction, no separate
// subscription/QoS/executor — and anchors the detour behind the boat so the
// tracker actually follows it. See unh_marine_navigation #59.
//
// Params (declared on the controller_server lifecycle node under <name>.*, so
// they are externally param-serviceable — rqt_reconfigure / ros2 param set):
//   <name>.primary_controller   (string) inner plugin type to load + wrap
//   <name>.max_deviation        corridor half-width (m); also the give-up bound
//   <name>.along_track_spacing  station spacing along the line (m)
//   <name>.lateral_resolution   cross-track search resolution (m)
//   <name>.line_following_weight        higher = hug the line
//   <name>.obstacle_avoidance_weight    higher = deviate more
//   <name>.smoothness_weight            higher = smoother detours
//   <name>.chatter_damping_weight       higher = steadier tick-to-tick
//   <name>.max_lateral_change           max cross-track change between stations (m)
//   <name>.anchor_behind_distance       (m) how far behind the boat to anchor the
//                                       detour entry to nominal (the #59 fix; 0 = at boat)
//   <name>.avoid_speed          (m/s) inner speed limit while deviating; 0 = off
class AvoidanceController : public nav2_core::Controller
{
public:
  AvoidanceController() = default;
  ~AvoidanceController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

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
  // Reshape the nominal plan around obstacles for the given robot pose (in the
  // costmap global frame). Returns the reshaped path in the same frame as the
  // stored nominal plan and writes the per-station offsets to last_offsets_ /
  // last_stations_. Returns the nominal plan unchanged when clear, blocked, or
  // when prerequisites (costmap, < 3 stations) are missing — the wrapper is a
  // path transformer, never a gate.
  nav_msgs::msg::Path reshapeAroundObstacles(const geometry_msgs::msg::PoseStamped & pose);

  // Read the live <name>.* tunables into params_ / station_step_ / avoid_speed_.
  void refreshParams();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("AvoidanceController")};
  rclcpp::Clock::SharedPtr clock_;

  // The pluginlib loader MUST outlive the loaded instance — a scope-local loader
  // unloads the shared library and the inner controller's vtable with it, so it
  // is a member here. See #59 plan / review.
  pluginlib::ClassLoader<nav2_core::Controller> primary_loader_{
    "nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr primary_;

  nav_msgs::msg::Path nominal_plan_;

  marine_nav_utilities::CorridorParams params_;
  double station_step_{2.0};
  double anchor_behind_distance_{4.0};
  double avoid_speed_{0.0};

  // Previous tick's per-station offsets, for the solver's temporal (chatter) term.
  std::vector<double> prev_offsets_;

  // The speed limit last handed down by the controller_server (setSpeedLimit),
  // forwarded to the inner controller. While deviating with avoid_speed_ > 0 we
  // override the inner limit with avoid_speed_, then restore this on the
  // deviating->clear transition. -1.0 == no server limit.
  double server_speed_limit_{-1.0};
  bool server_speed_is_percentage_{false};
  bool avoid_speed_active_{false};

  // Operator-feedback overlay (auto-discovered by CAMP); cleared on the
  // avoiding->clear transition that was_avoiding_ tracks.
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  bool was_avoiding_{false};
};

}  // namespace marine_nav_avoidance_controller

#endif  // MARINE_NAV_AVOIDANCE_CONTROLLER_AVOIDANCE_CONTROLLER_H
