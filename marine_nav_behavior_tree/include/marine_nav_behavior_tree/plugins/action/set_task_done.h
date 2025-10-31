#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_TASK_DONE_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_TASK_DONE_H

#include <behaviortree_cpp/bt_factory.h>

namespace marine_nav_behavior_tree
{

class SetTaskDone: public BT::SyncActionNode
{
public:
  SetTaskDone(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
