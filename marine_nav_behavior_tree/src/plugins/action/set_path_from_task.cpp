#include "marine_nav_behavior_tree/plugins/action/set_path_from_task.h"
#include "nav_msgs/msg/path.hpp"
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

  auto start_index = start_index_bb.value();
  auto end_index = end_index_bb.value();

  if(end_index < 0)
  {
    end_index = task->message().poses.size() + end_index;
  }

  const auto& poses = task->message().poses;

  nav_msgs::msg::Path path;

  for(std::size_t i = start_index; i <= end_index && i < poses.size(); i++)
  {
    path.poses.push_back(poses[i]);
  }
  if(!path.poses.empty())
  {
    path.header = path.poses.front().header;
  }

  setOutput("path", path);

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree
