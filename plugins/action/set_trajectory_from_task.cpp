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
    BT::InputPort<TaskPtr>("task", "{current_task}", "Task to get trajectory from"),
    BT::InputPort<int>("start_index", "0", "Index of the first pose in the trajectory"),
    BT::InputPort<int>("end_index", "-1", "Index of the last pose in the trajectory"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped> >("trajectory", "{current_navigation_trajectory}", "Trajectory to set"),

  };
}

BT::NodeStatus SetTrajectoryFromTask::tick()
{
  auto task_bb = getInput<TaskPtr>("task");
  if(!task_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [task]: ", task_bb.error() );
  }

  auto task = task_bb.value();

  auto start_index_bb = getInput<int>("start_index");
  if(!start_index_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [start_index]: ", start_index_bb.error() );
  }
  auto end_index_bb = getInput<int>("end_index");
  if(!end_index_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [end_index]: ", end_index_bb.error() );
  }

  auto start_index = start_index_bb.value();
  auto end_index = end_index_bb.value();

  if(end_index < 0)
  {
    end_index = task->message().poses.size() + end_index;
  }

  const auto& poses = task->message().poses;

  std::vector<geometry_msgs::msg::PoseStamped> trajectory;

  for(std::size_t i = start_index; i <= end_index && i < poses.size(); i++)
  {
    trajectory.push_back(poses[i]);
  }

  setOutput("trajectory", trajectory);
 
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation
