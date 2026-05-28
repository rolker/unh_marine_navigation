#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_TASK_FAILED_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SET_TASK_FAILED_H

#include <behaviortree_cpp/bt_factory.h>

namespace marine_nav_behavior_tree
{

// Records a task as attempted-but-failed: writes a structured "failed" marker into the
// task's status (surfaced to the operator/camp via the RunTasks heartbeat) and marks the
// task done so the mission advances. Distinct from SetTaskDone, which leaves status empty
// (a clean completion). Returns SUCCESS so a containing loop keeps running (skip-and-continue).
class SetTaskFailed: public BT::SyncActionNode
{
public:
  SetTaskFailed(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
