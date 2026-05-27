#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_PREDICT_STOPPING_POSE_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_PREDICT_STOPPING_POSE_H

#include <string>

#include <behaviortree_cpp/bt_factory.h>  // NOLINT(build/include_order) cpplint misclassifies third-party .h headers as "C system" by extension.
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace marine_nav_behavior_tree
{

// Projects the robot's momentum-coasting stop point. Given the current pose,
// the body-frame velocity (from the navigator's OdomSmoother), and a negative
// deceleration, it outputs the pose the robot would coast to under constant
// deceleration: stop = position + v̂·|v|²/(2·|decel|).
//
// Used to seed a Hover hold point that accounts for the boat's entry momentum
// instead of holding the live pose (which a boat arriving at speed sails past,
// then slowly drives back to — the overshoot fixed by #33). Ported from the
// pre-nav2 project11_navigation node of the same name; sources velocity from
// the OdomSmoother and pose from tf (both on the BT blackboard) rather than the
// removed project11_navigation::Context.
class PredictStoppingPose : public BT::SyncActionNode
{
public:
  PredictStoppingPose(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  // Pure projection, factored out so it is unit-testable without a tf or
  // blackboard fixture. `body_twist` is the velocity in the robot body frame;
  // it is rotated into `current`'s frame by `current.pose.orientation` (the
  // body→world rotation — the full 2D velocity, crabbing included). The result
  // is in the same frame as `current`. `deceleration` must be < 0; a value
  // >= 0, or a near-zero speed, returns `current` unchanged (no projection).
  static geometry_msgs::msg::PoseStamped projectStoppingPose(
    const geometry_msgs::msg::PoseStamped & current,
    const geometry_msgs::msg::Twist & body_twist,
    double deceleration);
};

}  // namespace marine_nav_behavior_tree

#endif  // MARINE_NAV_BEHAVIOR_TREE_ACTIONS_PREDICT_STOPPING_POSE_H
