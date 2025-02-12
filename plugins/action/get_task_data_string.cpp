#include "project11_navigation/plugins/action/get_task_data_string.h"

#include <project11_navigation/task.h>

namespace project11_navigation
{

GetTaskDataString::GetTaskDataString(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList GetTaskDataString::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Task> >("task", "{task}", "Task to get data from"),
    BT::InputPort<std::string>("key"),
    BT::InputPort<std::string>("default_value"),
    BT::OutputPort<std::string>("value")
  };
}

BT::NodeStatus GetTaskDataString::tick()
{
  auto task = getInput<std::shared_ptr<Task> >("task");
  if(!task)
  {
    throw BT::RuntimeError(name(), " missing required input [task]: ", task.error() );
  }

  auto key = getInput<std::string>("key");
  if(!key)
  {
    throw BT::RuntimeError(name(), " missing required input [key]: ", key.error() );
  }

  auto default_value = getInput<std::string>("default_value");
  if(!default_value)
  {
    throw BT::RuntimeError(name(), " missing required input [default_value]: ", default_value.error() );
  }

  auto value = default_value.value();
  if(task.value())
  {
    auto value_item = task.value()->dataItem(key.value());
    if(!value_item.IsNull())
      value = value_item.as<std::string>();
  }

  setOutput("value", value);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::GetTaskDataString>("GetTaskDataString");
}
