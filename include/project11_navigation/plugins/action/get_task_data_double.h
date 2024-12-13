#ifndef PROJECT11_NAVIGATION_ACTIONS_GET_TASK_DATA_DOUBLE_H
#define PROJECT11_NAVIGATION_ACTIONS_GET_TASK_DATA_DOUBLE_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class GetTaskDataDouble: public BT::SyncActionNode
{
public:
  GetTaskDataDouble(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
