#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_GET_SUB_PATH_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_GET_SUB_PATH_H

#include <behaviortree_cpp/bt_factory.h>
#include "nav_msgs/msg/path.hpp"

namespace marine_nav_behavior_tree
{

class GetSubPath: public BT::SyncActionNode
{
public:
  GetSubPath(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Input path as nav_msgs/msg/Path"),
      BT::InputPort<int>("start_index", "0", "Index of the first pose in the trajectory"),
      BT::InputPort<int>("end_index", "-1", "Index of the last pose in the trajectory"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Sub-path as nav_msgs/msg/Path"),
    };
  }

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
