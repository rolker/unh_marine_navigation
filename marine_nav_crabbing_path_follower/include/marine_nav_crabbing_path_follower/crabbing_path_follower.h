#ifndef MARINE_NAV_CRABBING_PATH_FOLLOWER_CRABBING_PATH_FOLLOWER_H
#define MARINE_NAV_CRABBING_PATH_FOLLOWER_CRABBING_PATH_FOLLOWER_H

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

  double desired_speed_;
  double speed_limit_ = -1.0;
  bool speed_limit_is_percentage_ = false;


  rclcpp::Duration transform_tolerance_ {0, 0};

  int current_segment_ = -1;
  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

  std::shared_ptr<control_toolbox::PidROS> pid_;
  rclcpp::Time last_update_time_;
  rclcpp::Duration pid_reset_threshold_ {1, 0};

  bool visualize_ = false;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
};


} // namespace marine_nav_crabbing_path_follower

#endif