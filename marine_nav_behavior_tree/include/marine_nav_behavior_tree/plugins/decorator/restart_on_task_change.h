#ifndef MARINE_NAV_BEHAVIOR_TREE__PLUGINS__DECORATOR__RESTART_ON_TASK_CHANGE_H_
#define MARINE_NAV_BEHAVIOR_TREE__PLUGINS__DECORATOR__RESTART_ON_TASK_CHANGE_H_

#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/time.hpp"

namespace marine_nav_behavior_tree
{

/**
 * @brief Decorator that restarts its child (halt + re-tick from the top) when a
 * watched task-update timestamp changes while the child is RUNNING.
 *
 * Motivation (#46): HoverTask is dispatched by a Switch keyed on task *type*. A
 * same-type re-command — a new hover_override, or a fresh hover task with a
 * different id — does not change the type, so the Switch never halts the hover
 * branch. The (plain Sequence) hover subtree then stays parked at its RUNNING
 * Hover child and never re-runs the goal-set Fallback / PredictStoppingPose, so
 * {hover_target} is never refreshed and the boat holds the stale spot. Wrapping
 * the hover Sequence in this decorator forces a clean re-entry (recompute the
 * target, re-send the hover goal) the moment the operator commands a new or
 * updated hover.
 *
 * The watched value is {current_task_update_time}. Task::update bumps that stamp
 * only when the task message actually changes (see marine_nav_tasks task.cpp), so
 * a benign feedback-loop re-send of an identical task does NOT trigger a restart.
 *
 * The baseline timestamp is (re)adopted on the first tick of each fresh entry —
 * i.e. after the node has been halted (type-change dispatch) or after the child
 * completed — so a type-driven re-entry never spuriously restarts on its own.
 */
class RestartOnTaskChange : public BT::DecoratorNode
{
public:
  RestartOnTaskChange(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  void halt() override;

private:
  BT::NodeStatus tick() override;

  bool have_baseline_{false};
  rclcpp::Time baseline_;
};

}  // namespace marine_nav_behavior_tree

#endif  // MARINE_NAV_BEHAVIOR_TREE__PLUGINS__DECORATOR__RESTART_ON_TASK_CHANGE_H_
