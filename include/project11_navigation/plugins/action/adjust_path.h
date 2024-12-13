#ifndef PROJECT11_NAVIGATION_ACTIONS_ADJUST_PATH_H
#define PROJECT11_NAVIGATION_ACTIONS_ADJUST_PATH_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class AdjustPath: public BT::SyncActionNode
{
public:
  AdjustPath(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


} // namespace project11_navigation

#endif
