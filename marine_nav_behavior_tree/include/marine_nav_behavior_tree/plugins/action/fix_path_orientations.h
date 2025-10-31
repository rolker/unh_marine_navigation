#ifndef MARINE_NAV_BEHAVIOR_TREE_PLUGINS_ACTION_FIX_PATH_ORIENTATIONS_H
#define MARINE_NAV_BEHAVIOR_TREE_PLUGINS_ACTION_FIX_PATH_ORIENTATIONS_H

#include <behaviortree_cpp/bt_factory.h>
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"


namespace marine_nav_behavior_tree
{

class FixPathOrientations: public BT::SyncActionNode
{
public:
  FixPathOrientations(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Input path as nav_msgs/msg/Path"),
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", "{tf_buffer}", "TF2 buffer"),
      BT::InputPort<std::string>("robot_frame", "{robot_frame}", "Robot base frame"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path with fixed orientations as nav_msgs/msg/Path"),
    };
  }

  BT::NodeStatus tick() override;
};

} // namespace marine_nav_behavior_tree

#endif
