#ifndef PROJECT11_NAVIGATION_CONTEXT_H
#define PROJECT11_NAVIGATION_CONTEXT_H

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "project11_navigation/environment.h"
#include "project11_navigation/navigator_settings.h"
#include "project11_navigation/robot.h"
#include "project11_navigation/robot_capabilities.h"
#include <mutex>

namespace project11_navigation
{

// Assembles the relevant data for accomplishing navigation tasks.
class Context
{
public:
  using Ptr = std::shared_ptr<Context>;
  using ConstPtr = std::shared_ptr<const Context>;

  Context(rclcpp_lifecycle::LifecycleNode::WeakPtr node);

  const Environment& environment() const;
  const Robot& robot() const;
  Robot& robot();
  const RobotCapabilities& robot_capabilities() const;
  const NavigatorSettings& navigator_settings() const;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer() const;
  geometry_msgs::msg::PoseStamped getPoseInFrame(std::string frame_id);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node() const;
private:
  Environment environment_;
  NavigatorSettings navigator_settings_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  Robot robot_;
  RobotCapabilities robot_capabilities_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

} // namespace project11_navigation

#endif
