#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_PATH_FROM_TASK_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_PATH_FROM_TASK_H

#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace marine_nav_behavior_tree
{

class SetPathFromTask: public BT::SyncActionNode
{
public:
  SetPathFromTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  // Build a Path from a pose vector by copying the index range
  // [start_index, end_index] (negative end_index counts from the end).
  // The outer header keeps the first pose's frame_id but zeros its stamp —
  // "latest" in TF lookups, the same idiom used in path_to_pose_vector.cpp
  // (see #23). Per-pose stamps stay untouched; downstream consumers
  // (crabbing_path_follower segment timing, marine_nav_utilities trajectory
  // computation) rely on them.
  static nav_msgs::msg::Path buildPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    int start_index,
    int end_index);
};

} // namespace marine_nav_behavior_tree

#endif
