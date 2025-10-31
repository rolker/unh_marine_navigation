#include "project11_navigation/plugins/condition/task_updated.h"

#include "project11_navigation/task.h"

namespace project11_navigation
{

TaskUpdated::TaskUpdated(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{
}

BT::PortsList TaskUpdated::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task", "{task}", "Task to check for updates"),
    BT::InputPort<rclcpp::Time>("last_update_time", "{last_update_time}", "Last update time to compare against"),
  };
}

BT::NodeStatus TaskUpdated::tick()
{
  auto task = getInput<TaskPtr>("task");
  if(!task)
  {
    throw BT::RuntimeError(name(), " missing required input [task]: ", task.error() );
  }
  auto last_update_time = getInput<rclcpp::Time>("last_update_time");
  if(!last_update_time)
  {
    throw BT::RuntimeError(name(), " missing required input [last_update_time]: ", last_update_time.error() );
  }

  if(task.value()->lastUpdateTime() > last_update_time.value())
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation
