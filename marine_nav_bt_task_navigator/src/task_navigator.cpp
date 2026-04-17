#include "marine_nav_bt_task_navigator/task_navigator.h"
#include "marine_nav_tasks/task.h"
#include "marine_nav_tasks/task_list.h"
#include "marine_nav_behavior_tree/bt_types.h"
#include "nav2_util/node_utils.hpp"

namespace marine_nav_bt_task_navigator
{

using marine_nav_tasks::TaskList;

bool TaskNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  auto node = parent_node.lock();
  RCLCPP_INFO_STREAM(node->get_logger(), "Configuring TaskNavigator");

  nav2_util::declare_parameter_if_not_declared(
    node, getName()+".task_list_blackboard_id", rclcpp::ParameterValue(std::string("task_list")));
  task_list_blackboard_id_ = node->get_parameter(getName()+".task_list_blackboard_id").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, getName()+".active_task_blackboard_id", rclcpp::ParameterValue(std::string("active_task_id")));
  active_task_blackboard_id_ = node->get_parameter(getName()+".active_task_blackboard_id").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, getName()+".debug", rclcpp::ParameterValue(false));
  debug_ = node->get_parameter(getName()+".debug").as_bool();

  clock_ = node->get_clock();

  marine_nav_behavior_tree::registerJsonDefinitions();

  auto bt_node = bt_action_server_->getBlackboard()->get<rclcpp::Node::SharedPtr>("node");
  if (!bt_node) {
    RCLCPP_ERROR(node->get_logger(), "TaskNavigator: Failed to get node from blackboard");
    return false;
  }

  bt_action_server_->getBlackboard()->set("robot_frame", feedback_utils_.robot_frame);

  return true;
}

bool TaskNavigator::activate()
{
  RCLCPP_INFO_STREAM(logger_, "Activating TaskNavigator");
  return true;
}

std::string TaskNavigator::getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_task_navigator_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("marine_nav_bt_task_navigator");
    node->declare_parameter<std::string>(
      "default_task_navigator_bt_xml",
      pkg_share_dir +
      "/behavior_trees/run_tasks.xml");
  }

  node->get_parameter("default_task_navigator_bt_xml", default_bt_xml_filename);

  RCLCPP_INFO_STREAM(node->get_logger(), "TaskNavigator: Default BT XML file: " << default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool TaskNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO_STREAM(logger_, "TaskNavigator: Received goal: " << goal->tasks.size() << " tasks");

  if(!bt_action_server_->loadBehaviorTree())
  {
    RCLCPP_ERROR(logger_, "Failed to load behavior tree");
    return false;
  }
  groot_.reset();
  groot_ = std::make_shared<BT::Groot2Publisher>(bt_action_server_->getTree());

  ros_topic_logger_.reset();
  auto bt_node = bt_action_server_->getBlackboard()->get<rclcpp::Node::SharedPtr>("node");
  if (bt_node) {
    ros_topic_logger_ = std::make_shared<nav2_behavior_tree::RosTopicLogger>(
      bt_node, bt_action_server_->getTree());
  }

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
    {
      feedback_msg->feedback.tasks = task_list->taskMessages();
    }
  }
  bt_action_server_->publishFeedback(feedback_msg);

  if (ros_topic_logger_) {
    ros_topic_logger_->flush();
  }
}

void TaskNavigator::onPreempt(typename ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO_STREAM(logger_, "TaskNavigator: Preempting goal");
  if(!initializeTaskList(goal))
  {
    bt_action_server_->terminatePendingGoal();
  }
  bt_action_server_->acceptPendingGoal();
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
  RCLCPP_INFO_STREAM(logger_, "TaskNavigator: Initializing task list");
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

} // namespace marine_nav_bt_task_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(marine_nav_bt_task_navigator::TaskNavigator, nav2_core::NavigatorBase)
