#ifndef PROJECT11_NAVIGATION_CONTEXT_H
#define PROJECT11_NAVIGATION_CONTEXT_H

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "project11_navigation/environment.h"
#include "project11_navigation/robot.h"

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

  std::shared_ptr<tf2_ros::Buffer> tfBuffer() const;
  geometry_msgs::msg::PoseStamped getPoseInFrame(std::string frame_id);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node() const;
private:
  Environment environment_;
  Robot robot_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

} // namespace project11_navigation

#endif
