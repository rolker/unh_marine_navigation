#ifndef PROJECT11_NAVIGATION_CONDITIONS_TASK_UPDATED_H
#define PROJECT11_NAVIGATION_CONDITIONS_TASK_UPDATED_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class TaskUpdated: public BT::ConditionNode
{
public:
  TaskUpdated(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
