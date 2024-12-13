#ifndef PROJECT11_NAVIGATION_ACTIONS_TASK_LIST_UPDATER_H
#define PROJECT11_NAVIGATION_ACTIONS_TASK_LIST_UPDATER_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class TaskListUpdater: public BT::SyncActionNode
{
public:
  TaskListUpdater(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
