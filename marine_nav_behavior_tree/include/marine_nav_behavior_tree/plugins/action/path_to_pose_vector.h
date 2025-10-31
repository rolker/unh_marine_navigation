#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_PATH_TO_POSE_VECTOR_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_PATH_TO_POSE_VECTOR_H

#include <behaviortree_cpp/bt_factory.h>

namespace marine_nav_behavior_tree
{

class PathToPoseVector: public BT::SyncActionNode
{
public:
  PathToPoseVector(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
