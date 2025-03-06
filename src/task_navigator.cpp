#include "project11_navigation/task_navigator.h"
#include "project11_navigation/task_list.h"
#include "project11_navigation/bt_types.h"
#include "nav2_util/node_utils.hpp"

namespace project11_navigation
{

bool TaskNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  auto node = parent_node.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, getName()+".task_list_blackboard_id", rclcpp::ParameterValue(std::string("task_list")));
  task_list_blackboard_id_ = node->get_parameter(getName()+".task_list_blackboard_id").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, getName()+".active_task_blackboard_id", rclcpp::ParameterValue(std::string("active_task_id")));
  active_task_blackboard_id_ = node->get_parameter(getName()+".active_task_blackboard_id").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, getName()+".debug", rclcpp::ParameterValue(false));
  debug_ = node->get_parameter(getName()+".debug").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(), "Debug: " << debug_);

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());
  clock_ = node->get_clock();

  registerJsonDefinitions();

  return true;
}

std::string TaskNavigator::getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_task_navigator_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("project11_navigation");
    node->declare_parameter<std::string>(
      "default_task_navigator_bt_xml",
      pkg_share_dir +
      "/behavior_trees/run_tasks.xml");
  }

  node->get_parameter("default_task_navigator_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool TaskNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  if(!bt_action_server_->loadBehaviorTree())
  {
    RCLCPP_ERROR(logger_, "Failed to load behavior tree");
    return false;
  }
  groot_.reset();
  groot_ = std::make_shared<BT::Groot2Publisher>(bt_action_server_->getTree());

  if(debug_)
  {
    RCLCPP_INFO(logger_, "Logging to console");
    console_logger_ = std::make_shared<BT::StdCoutLogger>(bt_action_server_->getTree());
  }

  return initializeTaskList(goal);
}

void TaskNavigator::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  auto current_nav_task = blackboard->getEntry(active_task_blackboard_id_);
  if(current_nav_task && current_nav_task->value.isString())
    feedback_msg->feedback.current_navigation_task = current_nav_task->value.cast<std::string>();

  auto task_list_entry = blackboard->getEntry(task_list_blackboard_id_);
  if(task_list_entry)
  {
    auto task_list = task_list_entry->value.cast<std::shared_ptr<TaskList> >();
    if(task_list)
      feedback_msg->feedback.tasks = task_list->taskMessages();
  }
  bt_action_server_->publishFeedback(feedback_msg);

}

void TaskNavigator::onPreempt(typename ActionT::Goal::ConstSharedPtr goal)
{
  if(!initializeTaskList(goal))
  {
    bt_action_server_->terminatePendingGoal();
  }
}

void TaskNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  auto blackboard = bt_action_server_->getBlackboard();
  auto task_list_entry = blackboard->getEntry("task_list");
  if(task_list_entry)
  {
    auto task_list = task_list_entry->value.cast<std::shared_ptr<TaskList> >();
    if(task_list)
      result->tasks = task_list->taskMessages();
  }
}

bool TaskNavigator::initializeTaskList(ActionT::Goal::ConstSharedPtr goal)
{
  auto blackboard = bt_action_server_->getBlackboard();
  if(goal->tasks.empty())
    blackboard->set(task_list_blackboard_id_, std::shared_ptr<TaskList>());
  else
  {
    std::shared_ptr<TaskList> task_list;
    blackboard->get(task_list_blackboard_id_, task_list);
    if(!task_list)
    {
      task_list = std::make_shared<TaskList>();
    }
    task_list->update(goal->tasks, clock_);
    blackboard->set(task_list_blackboard_id_, task_list);
  }

  return true;
}

} // namespace project11_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(project11_navigation::TaskNavigator, nav2_core::NavigatorBase)
