#ifndef PROJECT11_NAVIGATION_ACTIONS_PREDICT_STOPPING_POSE_H
#define PROJECT11_NAVIGATION_ACTIONS_PREDICT_STOPPING_POSE_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class PredictStoppingPose: public BT::SyncActionNode
{
public:
  PredictStoppingPose(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
