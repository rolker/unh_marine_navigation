#ifndef PROJECT11_NAVIGATION_NAVIGATOR_H
#define PROJECT11_NAVIGATION_NAVIGATOR_H

#include "rclcpp/rclcpp.hpp"
#include "marine_nav_interfaces/msg/task_information.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "marine_nav_interfaces/action/run_tasks.hpp"
#include "std_msgs/msg/string.hpp"

#include <project11_navigation/context.h>
#include <marine_nav_tasks/task_list.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_sqlite_logger.h>

namespace project11_navigation
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Action server that accepts tasks and executes them.
class Navigator: public rclcpp_lifecycle::LifecycleNode
{
public:
  using RunTasks = marine_nav_interfaces::action::RunTasks;
  using GoalHandleRunTasks = rclcpp_action::ServerGoalHandle<RunTasks>;

  Navigator(const std::string & node_name);

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunTasks::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleRunTasks> goal);
  void handleAccepted(const std::shared_ptr<GoalHandleRunTasks> goal_handle);

  /// Creates a Result message for the action server.
  std::shared_ptr<RunTasks::Result> generateResult();

  /// Clears the current goal
  void clearGoal();

  /// Updates and publishes nav state if it changed
  void updateNavigationState(std::string nav_state);


  void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr& msg);

  /// Builds and returns the Behavior Tree
  BT::Tree buildBehaviorTree();

  BT::BehaviorTreeFactory factory_;


  rclcpp_action::Server<RunTasks>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleRunTasks> goal_handle_;

  std::shared_ptr<Context> context_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr  display_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr navigation_state_publisher_;
  std::string last_navigation_state_;

  BT::Tree tree_;

  /// Global blackboard which can be seen by all sub-trees
  BT::Blackboard::Ptr blackboard_;

  std::shared_ptr<BT::Groot2Publisher> groot_;
  std::shared_ptr<BT::FileLogger2> logger_;
  std::shared_ptr<BT::SqliteLogger> sqlite_logger_;
  std::shared_ptr<BT::StdCoutLogger> console_logger_;
};

} // namespace project11_navigation

#endif
