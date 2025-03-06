#ifndef PROJECT11_NAVIGATION_ACTIONS_MULTIBEAM_COVERAGE_ACTION_H
#define PROJECT11_NAVIGATION_ACTIONS_MULTIBEAM_COVERAGE_ACTION_H

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <project11_nav_msgs/action/multibeam_coverage.hpp>
#include <project11_navigation/task.h>

namespace project11_navigation
{


class MultibeamCoverageActionSetGoal: public BT::SyncActionNode
{
public:
  MultibeamCoverageActionSetGoal(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class MultibeamCoverageActionUpdateTask: public BT::SyncActionNode
{
public:
  MultibeamCoverageActionUpdateTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class MultibeamCoverageActionCancel: public BT::SyncActionNode
{
public:
  MultibeamCoverageActionCancel(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class MultibeamCoverageActionDoneCondition: public BT::ConditionNode
{
public:
  MultibeamCoverageActionDoneCondition(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class MultibeamCoverageActionClient
{
  using MultibeamCoverage = project11_nav_msgs::action::MultibeamCoverage;
  using ClientPtr = rclcpp_action::Client<MultibeamCoverage>::SharedPtr;
  using GoalHandleMultibeamCoverage = rclcpp_action::ClientGoalHandle<MultibeamCoverage>;

public:
  MultibeamCoverageActionClient(std::shared_ptr<Task> task, const std::string& action_service, rclcpp_lifecycle::LifecycleNode::WeakPtr node);

  int lineCount() const;
  int lastLineNumber() const;
  bool done() const;

  ClientPtr actionClient() const;

private:
  friend class MultibeamCoverageActionUpdateTask;
  friend class MultibeamCoverageActionCancel;
  friend class MultibeamCoverageActionDoneCondition;

  ClientPtr action_client_;


  void actionResponseCallback(const GoalHandleMultibeamCoverage::SharedPtr & goal);
  void actionResultCallback(const GoalHandleMultibeamCoverage::WrappedResult & result);
  void actionFeedbackCallback(GoalHandleMultibeamCoverage::SharedPtr, const std::shared_ptr<const MultibeamCoverage::Feedback> feedback);

  std::vector<nav_msgs::msg::Path> survey_lines_;
  int last_line_number_ = -1;
  bool done_ = false;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

};


} // namespace project11_navigation


#endif
