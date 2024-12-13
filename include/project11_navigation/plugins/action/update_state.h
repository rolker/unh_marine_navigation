#ifndef PROJECT11_NAVIGATION_ACTIONS_UPDATE_STATE_H
#define PROJECT11_NAVIGATION_ACTIONS_UPDATE_STATE_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class UpdateState: public BT::SyncActionNode
{
public:
  UpdateState(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
