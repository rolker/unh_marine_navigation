#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_GET_SUB_PATH_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_GET_SUB_PATH_H

#include <behaviortree_cpp/bt_factory.h>
#include "nav_msgs/msg/path.hpp"

namespace marine_nav_behavior_tree
{

class GetSubPath: public BT::SyncActionNode
{
public:
  GetSubPath(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Input path as nav_msgs/msg/Path"),
      BT::InputPort<int>("start_index", "0", "Index of the first pose in the trajectory"),
      BT::InputPort<int>("end_index", "-1", "Index of the last pose in the trajectory"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Sub-path as nav_msgs/msg/Path"),
    };
  }

  BT::NodeStatus tick() override;

  // Extract a sub-range [start_index, end_index] (negative end_index counts
  // from the end) of input_path.poses into a new Path. The outer header
  // takes the first selected pose's frame_id but zeros its stamp — "latest"
  // in TF lookups, the same idiom used in path_to_pose_vector.cpp (see #23).
  // Per-pose stamps stay untouched.
  static nav_msgs::msg::Path buildSubPath(
    const nav_msgs::msg::Path& input_path,
    int start_index,
    int end_index);
};

} // namespace marine_nav_behavior_tree

#endif
