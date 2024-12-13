#ifndef PROJECT11_NAVIGATION_CONDITIONS_PLAN_NEEDED_H
#define PROJECT11_NAVIGATION_CONDITIONS_PLAN_NEEDED_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class PlanNeeded: public BT::ConditionNode
{
public:
  PlanNeeded(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
