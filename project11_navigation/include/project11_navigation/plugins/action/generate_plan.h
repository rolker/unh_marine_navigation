#ifndef PROJECT11_NAVIGATION_ACTIONS_GENERATE_PLAN_H
#define PROJECT11_NAVIGATION_ACTIONS_GENERATE_PLAN_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class GeneratePlan: public BT::SyncActionNode
{
public:
  GeneratePlan(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
