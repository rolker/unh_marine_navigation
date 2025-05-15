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
  auto blackboard = config().blackboard;
  auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_DEBUG_STREAM(node->get_logger(), "SetTaskDone  " << task.value()->message().id << " done:" << task.value()->done() << " pointer:" << task.value().get());

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

