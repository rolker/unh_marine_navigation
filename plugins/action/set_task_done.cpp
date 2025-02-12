#include <project11_navigation/plugins/action/set_task_done.h>
#include <project11_navigation/task.h>


namespace project11_navigation
{

SetTaskDone::SetTaskDone(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList SetTaskDone::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task", "{task}", "Task to set as done"),
  };
}

BT::NodeStatus SetTaskDone::tick()
{
  auto task = getInput<TaskPtr>("task");
  if(!task)
  {
    throw BT::RuntimeError("missing required input [task]: ", task.error() );
  }
  task.value()->setDone();

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::SetTaskDone>("SetTaskDone");
}

