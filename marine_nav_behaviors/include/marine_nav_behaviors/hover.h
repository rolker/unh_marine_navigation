#ifndef MARINE_NAV_BEHAVIORS_HOVER_H
#define MARINE_NAV_BEHAVIORS_HOVER_H

#include "nav2_behaviors/timed_behavior.hpp"
#include "marine_nav_interfaces/action/hover.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace marine_nav_behaviors
{

using HoverAction = marine_nav_interfaces::action::Hover;

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
  double maximum_rotation_speed_ = 0.5;
  geometry_msgs::msg::PoseStamped target_pose_;

  bool generate_visualization_ = false;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
  rclcpp::Time last_visualization_publish_time_;


};

} // namespace marine_nav_behaviors

#endif