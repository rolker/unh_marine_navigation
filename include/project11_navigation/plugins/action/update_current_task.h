#ifndef PROJECT11_NAVIGATION_ACTIONS_UPDATE_CURRENT_TASK_H
#define PROJECT11_NAVIGATION_ACTIONS_UPDATE_CURRENT_TASK_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class UpdateCurrentTask: public BT::SyncActionNode
{
public:
  UpdateCurrentTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
