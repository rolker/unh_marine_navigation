#include "marine_nav_behavior_tree/plugins/action/get_sub_tasks.h"
#include <marine_nav_tasks/task_list.h>
#include <marine_nav_tasks/task.h>

namespace marine_nav_behavior_tree
{

using marine_nav_tasks::Task;
using marine_nav_tasks::TaskList;

GetSubTasks::GetSubTasks(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList GetSubTasks::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Task> >("task", "{current_task}", "Task to get sub tasks from"),
    BT::OutputPort<std::shared_ptr<TaskList> >("sub_tasks", "{current_sub_tasks}", "Sub tasks of the input task"),
  };
}

BT::NodeStatus GetSubTasks::tick()
{
  auto task = getInput<std::shared_ptr<Task> >("task");
  if(!task)
  {
    throw BT::RuntimeError("GetSubTasks node named ",name(), " missing required input [task]: ", task.error() );
  }

  auto sub_tasks = std::make_shared<TaskList>(task.value()->children());
  setOutput("sub_tasks", sub_tasks);
  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree


