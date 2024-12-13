#ifndef PROJECT11_NAVIGATION_ACTIONS_GET_SUB_TASKS_H
#define PROJECT11_NAVIGATION_ACTIONS_GET_SUB_TASKS_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class GetSubTasks: public BT::SyncActionNode
{
public:
  GetSubTasks(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
