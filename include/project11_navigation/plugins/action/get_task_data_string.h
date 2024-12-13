#ifndef PROJECT11_NAVIGATION_ACTIONS_GET_TASK_DATA_STRING_H
#define PROJECT11_NAVIGATION_ACTIONS_GET_TASK_DATA_STRING_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class GetTaskDataString: public BT::SyncActionNode
{
public:
  GetTaskDataString(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
