#include <project11_navigation/plugins/action/set_trajectory_from_task.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <project11_navigation/task.h>


namespace project11_navigation
{

SetTrajectoryFromTask::SetTrajectoryFromTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList SetTrajectoryFromTask::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task"),
    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("trajectory"),
    BT::OutputPort<int>("current_segment")
  };
}

BT::NodeStatus SetTrajectoryFromTask::tick()
{
  auto task = getInput<TaskPtr>("task");
  if(!task)
  {
    throw BT::RuntimeError("missing required input [task]: ", task.error() );
  }

  auto trajectory = std::make_shared<std::vector<geometry_msgs::msg::PoseStamped> >(task.value()->message().poses);
  setOutput("trajectory", trajectory);
  setOutput("current_segment", 0);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::SetTrajectoryFromTask>("SetTrajectoryFromTask");
}
