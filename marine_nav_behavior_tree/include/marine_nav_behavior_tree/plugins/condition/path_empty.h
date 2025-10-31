#ifndef MARINE_NAV_BEHAVIOR_TREE_CONDITIONS_PATH_EMPTY_H
#define MARINE_NAV_BEHAVIOR_TREE_CONDITIONS_PATH_EMPTY_H

#include <behaviortree_cpp/bt_factory.h>
#include "nav_msgs/msg/path.hpp"

namespace marine_nav_behavior_tree
{

class PathEmpty: public BT::ConditionNode
{
public:
  PathEmpty(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "{path}", "Path to check")
    };
  }

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
