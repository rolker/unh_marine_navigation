#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_ADD_SUB_TASK_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_ADD_SUB_TASK_H

#include "behaviortree_cpp/bt_factory.h"

namespace marine_nav_behavior_tree
{

class AddSubTask: public BT::SyncActionNode
{
public:
  AddSubTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
