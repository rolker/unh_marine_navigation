#ifndef PROJECT11_NAVIGATION_ACTIONS_CLEAR_PATH_H
#define PROJECT11_NAVIGATION_ACTIONS_CLEAR_PATH_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class ClearPath: public BT::SyncActionNode
{
public:
  ClearPath(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
