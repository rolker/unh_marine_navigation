#include "marine_nav_behavior_tree/plugins/action/get_task_data_string.h"

#include <marine_nav_tasks/task.h>

namespace marine_nav_behavior_tree
{

using marine_nav_tasks::Task;
using marine_nav_tasks::TaskList;

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

} // namespace marine_nav_behavior_tree
