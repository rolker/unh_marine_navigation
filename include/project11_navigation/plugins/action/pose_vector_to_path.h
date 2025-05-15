#ifndef PROJECT11_NAVIGATION_ACTIONS_POSE_VECTOR_TO_PATH_H
#define PROJECT11_NAVIGATION_ACTIONS_POSE_VECTOR_TO_PATH_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class PoseVectorToPath: public BT::SyncActionNode
{
public:
  PoseVectorToPath(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif
