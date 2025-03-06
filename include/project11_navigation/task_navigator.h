#ifndef PROJECT11_NAVIGATION_TASK_NAVIGATOR_H
#define PROJECT11_NAVIGATION_TASK_NAVIGATOR_H

#include "nav2_core/behavior_tree_navigator.hpp"
#include "project11_nav_msgs/action/run_tasks.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

namespace project11_navigation
{

class TaskNavigator
  : public nav2_core::BehaviorTreeNavigator<
    project11_nav_msgs::action::RunTasks>
{
public:
  using ActionT = project11_nav_msgs::action::RunTasks;

  TaskNavigator()
  : BehaviorTreeNavigator()
  {
  }

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  std::string getName() override {return std::string("run_tasks");}

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  bool initializeTaskList(ActionT::Goal::ConstSharedPtr goal);
private:
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;
  std::string task_list_blackboard_id_;
  std::string active_task_blackboard_id_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<BT::Groot2Publisher> groot_;
  bool debug_ = false;
  std::shared_ptr<BT::StdCoutLogger> console_logger_;

};

} // namespace project11_navigation

#endif