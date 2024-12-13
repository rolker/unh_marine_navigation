#ifndef PROJECT11_NAVIGATION_CONDITIONS_GOAL_REACHED_H
#define PROJECT11_NAVIGATION_CONDITIONS_GOAL_REACHED_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class GoalReached: public BT::ConditionNode
{
public:
  GoalReached(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
