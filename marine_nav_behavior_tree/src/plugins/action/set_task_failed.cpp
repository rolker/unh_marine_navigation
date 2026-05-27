#include <marine_nav_behavior_tree/plugins/action/set_task_failed.h>
#include <marine_nav_tasks/task.h>
#include <yaml-cpp/yaml.h>


namespace marine_nav_behavior_tree
{

using TaskPtr = std::shared_ptr<marine_nav_tasks::Task>;

SetTaskFailed::SetTaskFailed(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList SetTaskFailed::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task", "{task}", "Task to record as attempted-but-failed"),
    BT::InputPort<std::string>("reason", "", "Short failure reason (e.g. follow_path error code)"),
    BT::InputPort<int>("attempts", 0, "Number of attempts made before giving up (0 = unset)"),
  };
}

BT::NodeStatus SetTaskFailed::tick()
{
  auto task = getInput<TaskPtr>("task");
  if(!task)
  {
    throw BT::RuntimeError("missing required input [task]: ", task.error() );
  }
  if(!task.value())
  {
    // No current task (e.g. mission cleared) — nothing to record. Succeed rather
    // than dereferencing a null Task pointer. Mirrors SetTaskDone's guard.
    return BT::NodeStatus::SUCCESS;
  }

  std::string reason;
  getInput<std::string>("reason", reason);
  int attempts = 0;
  getInput<int>("attempts", attempts);

  // Record attempted-but-failed in the task status. The status is a free-form YAML string
  // (TaskInformation.status) that rides the RunTasks heartbeat to the operator/camp; a
  // clean SetTaskDone leaves it empty, so a non-empty "failed" status distinguishes a
  // skipped/failed line from a completed one in the post-mission coverage record.
  YAML::Node status;
  status["outcome"] = "failed";
  if(!reason.empty())
    status["reason"] = reason;
  if(attempts > 0)
    status["attempts"] = attempts;
  task.value()->setStatus(status);

  // Mark done so the mission advances past this task (skip-and-continue); the status flags
  // it as failed-not-clean.
  task.value()->setDone();

  auto blackboard = config().blackboard;
  auto node = blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_ERROR_STREAM(node->get_logger(),
    "SetTaskFailed: task " << task.value()->message().id << " recorded failed"
    << (reason.empty() ? "" : (" (reason: " + reason + ")"))
    << "; advancing mission");

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree
