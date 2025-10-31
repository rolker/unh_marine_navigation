#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_POLYGON_FROM_TASK_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_POLYGON_FROM_TASK_H

#include <behaviortree_cpp/bt_factory.h>

namespace marine_nav_behavior_tree
{

class SetPolygonFromTask: public BT::SyncActionNode
{
public:
  SetPolygonFromTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
