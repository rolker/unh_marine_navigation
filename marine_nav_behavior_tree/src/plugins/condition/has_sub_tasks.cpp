#include "marine_nav_behavior_tree/plugins/condition/has_sub_tasks.h"

#include "marine_nav_tasks/task.h"

namespace marine_nav_behavior_tree
{

using TaskPtr = std::shared_ptr<marine_nav_tasks::Task>;

HasSubTasks::HasSubTasks(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList HasSubTasks::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task", "{current_task}", "Task to check for sub-tasks"),
    BT::InputPort<std::string>("task_type", "", "Optional type of tasks to check"),
    BT::InputPort<std::string>("task_id", "", "Optional ID of tasks to check"),
  };
}

BT::NodeStatus HasSubTasks::tick()
{
  auto task_type_bb = getInput<std::string>("task_type");
  std::string task_type;
  if(task_type_bb)
  {
    task_type = task_type_bb.value();
  }

  std::string task_id;
  auto task_id_bb = getInput<std::string>("task_id");
  if(task_id_bb)
  {
    task_id = task_id_bb.value();
  }

  auto task_bb = getInput<TaskPtr>("task");
  if(!task_bb)
  {
    throw BT::RuntimeError("HasSubTasks node named ",name(), " missing required input [task]: ", task_bb.error() );
  }

  auto task = task_bb.value();
  if(task)
  {
    if(!task_id.empty())
    {
      if(task_id.front() != '/')
      {
        task_id = task->getChildID(task_id);
      }
    }

    for (const auto& sub_task : task->children().tasks())
    {
      if(task_type.empty() || sub_task->message().type == task_type)
      {
        if(task_id.empty() || sub_task->message().id == task_id)
        {
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace marine_nav_behavior_tree
