#include <project11_navigation/plugins/action/set_pose_from_task.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <project11_navigation/task.h>
#include "project11_navigation/context.h"

namespace project11_navigation
{

SetPoseFromTask::SetPoseFromTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
}

BT::PortsList SetPoseFromTask::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task", "{current_task}", "Task to get pose from"),
    BT::InputPort<int>("pose_index", "0", "Index of the pose to set as output pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "{goal_pose}", "Pose to set"),
    BT::OutputPort<bool>("out_of_range_flag", "{index_out_of_range_flag}", "Flag to indicate if the pose index is out of range")
  };
}

BT::NodeStatus SetPoseFromTask::tick()
{
  auto task = getInput<TaskPtr>("task");
  if(!task)
  {
    throw BT::RuntimeError("missing required input [task]: ", task.error() );
  }

  auto pose_index = getInput<int>("pose_index");
  if(!pose_index)
  {
    throw BT::RuntimeError("missing required input [pose_index]: ", pose_index.error() );
  }

  int index = pose_index.value();
  if(index < 0) // python style count from the end
    index = task.value()->message().poses.size()+index;
  if(index < 0 || index >= task.value()->message().poses.size())
  {
    setOutput("out_of_range_flag", true);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("pose", task.value()->message().poses[pose_index.value()]);
  setOutput("out_of_range_flag", false);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

