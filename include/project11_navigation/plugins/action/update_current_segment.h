#ifndef PROJECT11_NAVIGATION_ACTIONS_UPDATE_CURRENT_SEGMENT_H
#define PROJECT11_NAVIGATION_ACTIONS_UPDATE_CURRENT_SEGMENT_H

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace project11_navigation
{

class UpdateCurrentSegment: public BT::SyncActionNode
{
public:
  UpdateCurrentSegment(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cross_track_error_publisher_;
};

} // namespace project11_navigation

#endif
