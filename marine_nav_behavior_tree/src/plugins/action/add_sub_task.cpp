#include "marine_nav_behavior_tree/plugins/action/add_sub_task.h"
#include "marine_nav_tasks/task.h"
#include "nav_msgs/msg/path.hpp"


namespace marine_nav_behavior_tree
{

AddSubTask::AddSubTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList AddSubTask::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<marine_nav_tasks::Task> >("parent_task", "{parent_task}", "Parent task to add sub-task to"),
    BT::InputPort<std::string >("sub_task_id", "{sub_task_id}", "ID of new sub-task to add to parent task"),
    BT::InputPort<std::string >("sub_task_type", "{sub_task_type}", "Type of new sub-task to add to parent task"),
    BT::InputPort<nav_msgs::msg::Path>("sub_task_path", "{sub_task_path}", "Path for the new sub-task to add to parent task"),
  };
}

BT::NodeStatus AddSubTask::tick()
{
  auto parent_task_bb = getInput<std::shared_ptr<marine_nav_tasks::Task> >("parent_task");
  if(!parent_task_bb)
  {
    throw BT::RuntimeError("AddSubTask node named ",name(), " missing required input [parent_task]: ", parent_task_bb.error() );
  }
  auto parent_task = parent_task_bb.value();

  auto sub_task_id_bb = getInput<std::string>("sub_task_id");
  if(!sub_task_id_bb)
  {
    throw BT::RuntimeError("AddSubTask node named ",name(), " missing required input [sub_task_id]: ", sub_task_id_bb.error() );
  }
  auto sub_task_id = sub_task_id_bb.value();

  auto sub_task_type_bb = getInput<std::string>("sub_task_type");
  if(!sub_task_type_bb)
  {
    throw BT::RuntimeError("AddSubTask node named ",name(), " missing required input [sub_task_type]: ", sub_task_type_bb.error() );
  }
  auto sub_task_type = sub_task_type_bb.value();

  auto sub_task_path_bb = getInput<nav_msgs::msg::Path>("sub_task_path");
  if(!sub_task_path_bb)
  {
    throw BT::RuntimeError("AddSubTask node named ",name(), " missing required input [sub_task_path]: ", sub_task_path_bb.error() );
  }
  auto sub_task_path = sub_task_path_bb.value();

  auto new_sub_task = parent_task->createChildTaskBefore({}, sub_task_type);

  marine_nav_interfaces::msg::TaskInformation sub_task_msg;
  sub_task_msg.type = sub_task_type;
  sub_task_msg.poses = sub_task_path.poses;
  new_sub_task->update(sub_task_msg, false);
  parent_task->setChildID(new_sub_task, sub_task_id);

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree
