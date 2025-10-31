#include <marine_nav_behavior_tree/plugins/action/set_task_done.h>
#include <marine_nav_tasks/task.h>


namespace marine_nav_behavior_tree
{

using TaskPtr = std::shared_ptr<marine_nav_tasks::Task>;

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
  auto blackboard = config().blackboard;
  auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_DEBUG_STREAM(node->get_logger(), "SetTaskDone  " << task.value()->message().id << " done:" << task.value()->done() << " pointer:" << task.value().get());

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree

