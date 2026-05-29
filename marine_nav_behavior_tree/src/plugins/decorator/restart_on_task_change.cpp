#include "marine_nav_behavior_tree/plugins/decorator/restart_on_task_change.h"

namespace marine_nav_behavior_tree
{

RestartOnTaskChange::RestartOnTaskChange(
  const std::string & name, const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{
}

BT::PortsList RestartOnTaskChange::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Time>(
      "task_update_time",
      "Timestamp of the current task's last update (wire to "
      "{current_task_update_time}). When it changes while the child is RUNNING, "
      "the child is halted and re-ticked from the top so a same-type task "
      "re-command takes effect.")
  };
}

void RestartOnTaskChange::halt()
{
  // Drop the baseline so the next fresh entry re-adopts the then-current stamp
  // rather than comparing against a stale one from a prior engagement.
  have_baseline_ = false;
  BT::DecoratorNode::halt();
}

BT::NodeStatus RestartOnTaskChange::tick()
{
  rclcpp::Time current;
  const bool have_input = static_cast<bool>(getInput("task_update_time", current));

  if (have_input) {
    if (!have_baseline_) {
      // Fresh entry (first tick, or first after a halt / completed child):
      // adopt the current stamp as the baseline; do not restart.
      baseline_ = current;
      have_baseline_ = true;
    } else if (current != baseline_) {
      // The task changed under us without a type-driven halt (same-type
      // re-command). Force a clean re-entry of the child subtree: haltChild()
      // resets a plain Sequence to its first child and cancels any running
      // action, so the subsequent tick recomputes the goal and re-sends it.
      baseline_ = current;
      haltChild();
    }
  }

  setStatus(BT::NodeStatus::RUNNING);
  const BT::NodeStatus child_status = child_node_->executeTick();

  // Once the child settles (SUCCESS/FAILURE), forget the baseline so a later
  // re-engagement adopts a fresh one instead of comparing across a gap.
  if (child_status != BT::NodeStatus::RUNNING) {
    have_baseline_ = false;
  }

  return child_status;
}

}  // namespace marine_nav_behavior_tree
