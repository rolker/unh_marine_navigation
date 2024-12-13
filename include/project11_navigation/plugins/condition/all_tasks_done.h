#ifndef PROJECT11_NAVIGATION_CONDITIONS_ALL_TASKS_DONE_H
#define PROJECT11_NAVIGATION_CONDITIONS_ALL_TASKS_DONE_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class AllTasksDone: public BT::ConditionNode
{
public:
  AllTasksDone(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
