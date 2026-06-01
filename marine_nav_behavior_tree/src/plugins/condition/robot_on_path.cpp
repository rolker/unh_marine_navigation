#include "marine_nav_behavior_tree/plugins/condition/robot_on_path.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "nav2_util/robot_utils.hpp"

namespace marine_nav_behavior_tree
{

RobotOnPath::RobotOnPath(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

namespace
{
// Distance from point (px, py) to the segment [(ax, ay), (bx, by)] in the XY plane.
double distancePointToSegment(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double seg_len_sq = dx * dx + dy * dy;
  double t = 0.0;
  if (seg_len_sq > 0.0) {
    // Projection parameter of the point onto the segment, clamped to [0, 1] so a
    // point beyond either end measures to the nearer endpoint.
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq;
    t = std::max(0.0, std::min(1.0, t));
  }
  const double cx = ax + t * dx;
  const double cy = ay + t * dy;
  return std::hypot(px - cx, py - cy);
}
}  // namespace

double RobotOnPath::minDistanceToPath(
  const geometry_msgs::msg::Point & point, const nav_msgs::msg::Path & path)
{
  const auto & poses = path.poses;
  if (poses.empty()) {
    return -1.0;
  }
  if (poses.size() == 1) {
    return std::hypot(
      point.x - poses.front().pose.position.x,
      point.y - poses.front().pose.position.y);
  }
  double min_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    const double d = distancePointToSegment(
      point.x, point.y,
      poses[i].pose.position.x, poses[i].pose.position.y,
      poses[i + 1].pose.position.x, poses[i + 1].pose.position.y);
    min_dist = std::min(min_dist, d);
  }
  return min_dist;
}

BT::NodeStatus RobotOnPath::tick()
{
  auto path_bb = getInput<nav_msgs::msg::Path>("path");
  if (!path_bb) {
    throw BT::RuntimeError(name(), " missing required input [path]: ", path_bb.error());
  }
  auto threshold_bb = getInput<double>("threshold");
  if (!threshold_bb) {
    throw BT::RuntimeError(name(), " missing required input [threshold]: ", threshold_bb.error());
  }
  auto tf_buffer_bb = getInput<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  if (!tf_buffer_bb) {
    throw BT::RuntimeError(name(), " missing required input [tf_buffer]: ", tf_buffer_bb.error());
  }
  auto robot_frame_bb = getInput<std::string>("robot_frame");
  if (!robot_frame_bb) {
    throw BT::RuntimeError(name(), " missing required input [robot_frame]: ", robot_frame_bb.error());
  }

  const auto & path = path_bb.value();
  if (path.poses.empty()) {
    // No line to be "on" → fall back to the lead-in (which will also be a no-op).
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_buffer_bb.value(), path.header.frame_id, robot_frame_bb.value()))
  {
    // Cannot determine the robot pose → fall back to the lead-in (transit to start).
    return BT::NodeStatus::FAILURE;
  }

  const double dist = minDistanceToPath(current_pose.pose.position, path);
  if (dist >= 0.0 && dist <= threshold_bb.value()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace marine_nav_behavior_tree
