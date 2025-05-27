#ifndef PROJECT11_NAVIGATION_BEHAVIORS_HOVER_H
#define PROJECT11_NAVIGATION_BEHAVIORS_HOVER_H

#include "nav2_behaviors/timed_behavior.hpp"
#include "project11_nav_msgs/action/hover.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace project11_navigation
{

using HoverAction = project11_nav_msgs::action::Hover;

class Hover : public nav2_behaviors::TimedBehavior<HoverAction>
{
public:
  Hover();

  nav2_behaviors::ResultStatus onRun(const std::shared_ptr<const HoverAction::Goal> goal) override;

  nav2_behaviors::ResultStatus onCycleUpdate() override;

  void onConfigure() override;

  nav2_core::CostmapInfoType getResourceInfo() override {return nav2_core::CostmapInfoType::LOCAL;}

protected:
  void publish_visualization(rclcpp::Time time);

  double minimum_radius_ = 1.0;
  double maximum_radius_ = 2.0;
  double minimum_speed_ = 0.0;
  double maximum_speed_ = 1.0;
  double deceleration_ = -0.5;
  double maximum_rotation_speed_ = 0.5;
  geometry_msgs::msg::PoseStamped target_pose_;

  bool generate_visualization_ = false;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr visualization_publisher_;


};

} // namespace project11_navigation

#endif