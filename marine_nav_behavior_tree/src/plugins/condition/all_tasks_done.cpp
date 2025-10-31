#include <marine_nav_behavior_tree/plugins/condition/all_tasks_done.h>

#include <marine_nav_tasks/task.h>

namespace marine_nav_behavior_tree
{

using TaskListPtr = std::shared_ptr<marine_nav_tasks::TaskList>;

AllTasksDone::AllTasksDone(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{

}

BT::PortsList AllTasksDone::providedPorts()
{
  return {
    BT::InputPort<TaskListPtr>("task_list", "{task_list}", "List of tasks to check")
  };
}

BT::NodeStatus AllTasksDone::tick()
{
  auto tasks = getInput<TaskListPtr>("task_list");
  if(!tasks)
  {
    throw BT::RuntimeError("AllTasksDone node named ",name(), " missing required input [task_list]: ", tasks.error() );    
  }

  if(tasks.value()->allDone())
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

} // namespace marine_nav_behavior_tree
