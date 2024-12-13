#include <project11_navigation/plugins/condition/all_tasks_done.h>

#include <project11_navigation/task.h>

namespace project11_navigation
{

AllTasksDone::AllTasksDone(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{

}

BT::PortsList AllTasksDone::providedPorts()
{
  return {
    BT::InputPort<TaskListPtr>("task_list")
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

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::AllTasksDone>("AllTasksDoneCondition");
}
