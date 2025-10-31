#ifndef PROJECT11_NAVIGATION_ACTIONS_SET_TRAJECTORY_FROM_TASK_H
#define PROJECT11_NAVIGATION_ACTIONS_SET_TRAJECTORY_FROM_TASK_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class SetTrajectoryFromTask: public BT::SyncActionNode
{
public:
  SetTrajectoryFromTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
