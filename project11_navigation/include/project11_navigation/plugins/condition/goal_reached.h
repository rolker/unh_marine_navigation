#ifndef PROJECT11_NAVIGATION_CONDITIONS_GOAL_REACHED_H
#define PROJECT11_NAVIGATION_CONDITIONS_GOAL_REACHED_H

#include <behaviortree_cpp/bt_factory.h>
#include "tf2_ros/buffer.h"

namespace project11_navigation
{

class GoalReached: public BT::ConditionNode
{
public:
  GoalReached(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

} // namespace project11_navigation

#endif
