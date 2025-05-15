#ifndef PROJECT11_NAVIGATION_CONDITIONS_PATH_EMPTY_H
#define PROJECT11_NAVIGATION_CONDITIONS_PATH_EMPTY_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class PathEmpty: public BT::ConditionNode
{
public:
  PathEmpty(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
