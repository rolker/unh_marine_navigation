#include "marine_nav_behavior_tree/plugins/action/set_path_from_task.h"

#include "builtin_interfaces/msg/time.hpp"
#include "marine_nav_tasks/task.h"


namespace marine_nav_behavior_tree
{

using TaskPtr = std::shared_ptr<marine_nav_tasks::Task>;


SetPathFromTask::SetPathFromTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList SetPathFromTask::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task", "{current_task}", "Task to get navigation path from"),
    BT::InputPort<int>("start_index", "0", "Index of the first pose in the navigation path"),
    BT::InputPort<int>("end_index", "-1", "Index of the last pose in the navigation path"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{current_navigation_path}", "Navigation path to set"),

  };
}

BT::NodeStatus SetPathFromTask::tick()
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

  setOutput("path", buildPath(task->message().poses, start_index_bb.value(), end_index_bb.value()));

  return BT::NodeStatus::SUCCESS;
}

nav_msgs::msg::Path SetPathFromTask::buildPath(
  const std::vector<geometry_msgs::msg::PoseStamped>& poses,
  int start_index,
  int end_index)
{
  if(end_index < 0)
  {
    end_index = static_cast<int>(poses.size()) + end_index;
  }

  nav_msgs::msg::Path path;

  for(std::size_t i = start_index; i <= static_cast<std::size_t>(end_index) && i < poses.size(); i++)
  {
    path.poses.push_back(poses[i]);
  }
  if(!path.poses.empty())
  {
    path.header.frame_id = path.poses.front().header.frame_id;
    // Zero stamp = "latest" in TF lookups; mirrors path_to_pose_vector.cpp's idiom.
    // Per-pose stamps stay untouched (load-bearing downstream — see header). For #23.
    path.header.stamp = builtin_interfaces::msg::Time();
  }
  return path;
}

} // namespace marine_nav_behavior_tree
