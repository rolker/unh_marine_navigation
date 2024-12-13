#ifndef PROJECT11_NAVIGATION_ACTIONS_SET_TASK_DONE_H
#define PROJECT11_NAVIGATION_ACTIONS_SET_TASK_DONE_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class SetTaskDone: public BT::SyncActionNode
{
public:
  SetTaskDone(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
