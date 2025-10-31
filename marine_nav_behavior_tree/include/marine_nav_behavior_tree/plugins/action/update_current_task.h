#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_UPDATE_CURRENT_TASK_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_UPDATE_CURRENT_TASK_H

#include <behaviortree_cpp/bt_factory.h>

namespace marine_nav_behavior_tree
{

class UpdateCurrentTask: public BT::SyncActionNode
{
public:
  UpdateCurrentTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
