#include "marine_nav_behavior_tree/plugins/action/predict_stopping_pose.h"

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>  // NOLINT(build/include_order)
#include <tf2/LinearMath/Vector3.h>  // NOLINT(build/include_order)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace marine_nav_behavior_tree
{

PredictStoppingPose::PredictStoppingPose(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PredictStoppingPose::providedPorts()
{
  return {
    BT::InputPort<double>(
      "deceleration",
      "Deceleration in m/s^2 (must be < 0) used to project the coasting stop "
      "point. Typically wired to the navigator's {default_deceleration}."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "pose",
      "Predicted momentum-coasting stop pose, in the odometry frame the velocity "
      "is reported in (the behavior_server's local_frame); falls back to the "
      "navigator's global frame only before any odometry has arrived.")
  };
}

geometry_msgs::msg::PoseStamped PredictStoppingPose::projectStoppingPose(
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::Twist & body_twist,
  double deceleration)
{
  geometry_msgs::msg::PoseStamped stop = current;

  // A non-negative "deceleration" can't bring the boat to rest: treat it as
  // a misconfiguration and hold the current pose rather than projecting to
  // infinity / the wrong direction.
  if (deceleration >= 0.0) {
    return stop;
  }

  tf2::Vector3 motion;
  tf2::fromMsg(body_twist.linear, motion);  // body frame (forward + crabbing)

  tf2::Quaternion orientation;
  tf2::fromMsg(current.pose.orientation, orientation);
  motion = tf2::quatRotate(orientation, motion);  // body -> current's frame

  const double speed = motion.length();
  if (speed < 1.0e-3) {
    return stop;  // effectively stationary: no projection
  }

  // Stopping distance under constant deceleration is d = v^2 / (2|a|). As a
  // vector along the velocity direction this is motion * (|v|/|a|) / 2, since
  // motion already carries the v̂ direction and |v| magnitude:
  //   motion * (stop_time / 2) = v̂·|v| * (|v|/|a|)/2 = v̂ · v^2/(2|a|).
  const double stop_time = -speed / deceleration;  // |v| / |a|  (deceleration < 0)
  motion *= (stop_time / 2.0);

  stop.pose.position.x += motion.x();
  stop.pose.position.y += motion.y();
  stop.pose.position.z += motion.z();
  return stop;
}

BT::NodeStatus PredictStoppingPose::tick()
{
  auto deceleration = getInput<double>("deceleration");
  if (!deceleration) {
    throw BT::RuntimeError(
      name(), " missing required input [deceleration]: ", deceleration.error());
  }

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto tf_buffer =
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto odom_smoother =
    config().blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother");
  auto robot_frame = config().blackboard->get<std::string>("robot_frame");

  // Project the stop point in the odometry frame the velocity is reported in
  // (this is the behavior_server's local_frame, e.g. <tf_prefix>/odom). Holding
  // station there lets Hover consume the result directly with no per-cycle map
  // transform, and avoids making Hover depend on a global (map) transform it
  // never otherwise needs — an unavailable map would otherwise abort the whole
  // HoverTask. Fall back to the navigator's global_frame only if no odom has
  // arrived yet (zero velocity then ⇒ the projection holds the current pose).
  std::string projection_frame = odom_smoother->getTwistStamped().header.frame_id;
  if (projection_frame.empty())
  {
    projection_frame = config().blackboard->get<std::string>("global_frame");
  }

  geometry_msgs::msg::PoseStamped current;
  if (!nav2_util::getCurrentPose(
      current, *tf_buffer, projection_frame, robot_frame, 0.1))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "PredictStoppingPose: current robot pose unavailable (%s -> %s)",
      projection_frame.c_str(), robot_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // OdomSmoother returns the smoothed velocity in the robot body frame.
  const auto stop = projectStoppingPose(
    current, odom_smoother->getTwist(), deceleration.value());
  setOutput("pose", stop);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace marine_nav_behavior_tree
