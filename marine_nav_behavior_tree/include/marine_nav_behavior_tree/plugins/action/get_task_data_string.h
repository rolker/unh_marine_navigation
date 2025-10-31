#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_GET_TASK_DATA_STRING_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_GET_TASK_DATA_STRING_H

#include <behaviortree_cpp/bt_factory.h>

namespace marine_nav_behavior_tree
{

class GetTaskDataString: public BT::SyncActionNode
{
public:
  GetTaskDataString(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
