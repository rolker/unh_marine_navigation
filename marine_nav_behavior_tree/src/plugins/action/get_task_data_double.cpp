#include "marine_nav_behavior_tree/plugins/action/get_task_data_double.h"

#include <marine_nav_tasks/task.h>

namespace marine_nav_behavior_tree
{

using marine_nav_tasks::Task;
using marine_nav_tasks::TaskList;


GetTaskDataDouble::GetTaskDataDouble(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList GetTaskDataDouble::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Task> >("task", "{task}", "Task to get data from"),
    BT::InputPort<std::string>("key"),
    BT::InputPort<double>("default_value"),
    BT::OutputPort<double>("value")
  };
}

BT::NodeStatus GetTaskDataDouble::tick()
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

  auto default_value = getInput<double>("default_value");
  if(!default_value)
  {
    throw BT::RuntimeError(name(), " missing required input [default_value]: ", default_value.error() );
  }

  auto value = default_value.value();
  if(task.value())
  {
    auto value_item = task.value()->dataItem(key.value());
    if(!value_item.IsNull())
      value = value_item.as<double>();
  }

  setOutput("value", value);

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree

