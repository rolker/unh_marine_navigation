#ifndef MARINE_NAV_BEHAVIOR_TREE_CONDITIONS_ROBOT_ON_PATH_H
#define MARINE_NAV_BEHAVIOR_TREE_CONDITIONS_ROBOT_ON_PATH_H

#include <behaviortree_cpp/bt_factory.h>
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"

namespace marine_nav_behavior_tree
{

/**
 * @brief Condition that succeeds when the robot is within `threshold` metres of
 * `path` (perpendicular distance to the nearest segment of the polyline).
 *
 * Motivation (#52): `TransitAndSurveyLine` unconditionally leads in to the survey
 * line's first waypoint on every entry. After an override (hover park / goto
 * nudge) halts and the survey-line case re-enters, that lead-in drives the boat
 * back to the line start and re-runs the whole line. Gating the lead-in on this
 * condition lets a resume skip the transit when the boat is already on the line —
 * the path-following controller (`CrabbingPathFollower`) then resumes from the
 * boat's current position. On a genuine fresh entry the boat is not yet on the
 * line, so the lead-in runs as before.
 *
 * Degrades safely: a missing TF (cannot determine the robot pose) or an empty
 * path returns FAILURE, preserving the existing transit-to-start behaviour.
 */
class RobotOnPath : public BT::ConditionNode
{
public:
  RobotOnPath(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "{survey_path}", "Path to test proximity to"),
      BT::InputPort<double>("threshold", "Maximum distance (m) from the path to count as 'on' it"),
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", "{tf_buffer}", "TF2 buffer"),
      BT::InputPort<std::string>("robot_frame", "{robot_frame}", "Robot base frame"),
    };
  }

  BT::NodeStatus tick() override;

  /// Minimum distance from `point` to the polyline through `path.poses`, in the
  /// XY plane. Returns -1.0 for an empty path (no meaningful distance). Static so
  /// the geometry can be unit-tested without a TF fixture.
  static double minDistanceToPath(
    const geometry_msgs::msg::Point & point, const nav_msgs::msg::Path & path);
};

}  // namespace marine_nav_behavior_tree

#endif
