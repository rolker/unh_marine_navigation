#include <project11_navigation/plugins/action/multibeam_coverage_action.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

MultibeamCoverageActionClient::MultibeamCoverageActionClient(std::shared_ptr<Task> task, const std::string& action_service, rclcpp::Node::SharedPtr node):
  node_(node)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Creating a client for " << action_service);
  action_client_ = rclcpp_action::create_client<MultibeamCoverage>(node_, action_service);

  if(!action_client_->wait_for_action_server())
    RCLCPP_INFO_STREAM(node_->get_logger(), "Timeout waiting for action server: " << action_service);

  auto goal = MultibeamCoverage::Goal();
  for(auto vertex: task->message().poses)
  {
    if(goal.survey_area.polygon.points.empty())
      goal.survey_area.header = vertex.header;
    geometry_msgs::msg::Point32 p;
    p.x = vertex.pose.position.x;
    p.y = vertex.pose.position.y;
    p.z = vertex.pose.position.z;
    goal.survey_area.polygon.points.push_back(p);
  }

  auto send_goal_options = rclcpp_action::Client<MultibeamCoverage>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const GoalHandleMultibeamCoverage::SharedPtr & goal_handle){this->actionResponseCallback(goal_handle);};

  send_goal_options.feedback_callback = [this](GoalHandleMultibeamCoverage::SharedPtr handle, const std::shared_ptr<const MultibeamCoverage::Feedback> feedback){this->actionFeedbackCallback(handle, feedback);};

  send_goal_options.result_callback = [this](const GoalHandleMultibeamCoverage::WrappedResult& result){this->actionResultCallback(result);};

  action_client_->async_send_goal(goal, send_goal_options);
}

void MultibeamCoverageActionClient::actionResultCallback(const GoalHandleMultibeamCoverage::WrappedResult & result)
{
  done_ = true;
}

void MultibeamCoverageActionClient::actionResponseCallback(const GoalHandleMultibeamCoverage::SharedPtr & goal)
{
  if(!goal)
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Multibeam coverage goal was rejected by the server");
  else
    RCLCPP_INFO_STREAM(node_->get_logger(), "Multibeam Coverage Action active");
}

void MultibeamCoverageActionClient::actionFeedbackCallback(GoalHandleMultibeamCoverage::SharedPtr, const std::shared_ptr<const MultibeamCoverage::Feedback> feedback)
{
  if(feedback->line_number != last_line_number_)
  {
    survey_lines_.push_back(feedback->current_line);
    last_line_number_ = feedback->line_number;
    adjustPathOrientations(survey_lines_.back().poses);
  }
}

int MultibeamCoverageActionClient::lineCount() const
{
  return survey_lines_.size();
}

int MultibeamCoverageActionClient::lastLineNumber() const
{
  return last_line_number_;
}

bool MultibeamCoverageActionClient::done() const
{
  return done_;
}

MultibeamCoverageActionClient::ClientPtr MultibeamCoverageActionClient::actionClient() const
{
  return action_client_;
}



// *** SetGoal ***


MultibeamCoverageActionSetGoal::MultibeamCoverageActionSetGoal(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList MultibeamCoverageActionSetGoal::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Task> >("task", "{task}", "Survey area Task describing area to survey"),
    BT::InputPort<std::string>("ros_action"),
    BT::OutputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client", "{multibeam_coverage_action_client}", "Multibeam coverage action client")
  };
}

BT::NodeStatus MultibeamCoverageActionSetGoal::tick()
{
  auto task_bb = getInput<std::shared_ptr<Task> >("task");
  if(!task_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageAction node named ",name(), " missing required input [task]: ", task_bb.error() );
  }
  auto task = task_bb.value();
  if(task)
  {
    auto ros_action = getInput<std::string>("ros_action");
    if(!ros_action)
    {
      throw BT::RuntimeError("MultibeamCoverageAction node named ",name(), " missing required input [ros_action]: ", ros_action.error() );
    }

    setOutput("action_client", std::make_shared<MultibeamCoverageActionClient>(task, ros_action.value(), node_));
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// *** UpdateTask ***

MultibeamCoverageActionUpdateTask::MultibeamCoverageActionUpdateTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList MultibeamCoverageActionUpdateTask::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client", "{multibeam_coverage_action_client}", "Multibeam coverage action client"),
    BT::InputPort<std::shared_ptr<Task> >("task", "{task}", "Survey area Task describing area to survey"),
  };
}

BT::NodeStatus MultibeamCoverageActionUpdateTask::tick()
{
  auto action_client_bb = getInput<std::shared_ptr<MultibeamCoverageActionClient> >("action_client");
  if(!action_client_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionUpdateTask node named ",name(), " missing required input [action_client]: ", action_client_bb.error() );
  }

  auto task_bb = getInput<std::shared_ptr<Task> >("task");
  if(!task_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionUpdateTask node named ",name(), " missing required input [task]: ", task_bb.error() );
  }
  auto action_client = action_client_bb.value();
  auto task = task_bb.value();
  if(action_client && task)
  {
    while(action_client->survey_lines_.size() > task->children().tasks().size())
    {
      auto line_index = task->children().tasks().size();
      auto new_child_task = task->createChildTaskBefore();
      std::stringstream task_name;
      task_name << "line_" << line_index;
      task->setChildID(new_child_task, task_name.str());
      auto info = new_child_task->message();
      info.type = "survey_line";
      info.poses = action_client->survey_lines_[line_index].poses;
      new_child_task->update(info);
      RCLCPP_INFO_STREAM(node_->get_logger(), "New task: " <<  project11_nav_msgs::msg::to_yaml(new_child_task->message()));
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// *** Cancel ***

MultibeamCoverageActionCancel::MultibeamCoverageActionCancel(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  
}

BT::PortsList MultibeamCoverageActionCancel::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client", "{multibeam_coverage_action_client}", "Multibeam coverage action client")
  };
}


BT::NodeStatus MultibeamCoverageActionCancel::tick()
{
  auto action_client_bb = getInput<std::shared_ptr<MultibeamCoverageActionClient> >("action_client");
  if(!action_client_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionCancel node named ",name(), " missing required input [action_client]: ", action_client_bb.error() );
  }
  action_client_bb.value()->action_client_->async_cancel_all_goals();
  return BT::NodeStatus::SUCCESS;
}

// *** DoneCondition ***

MultibeamCoverageActionDoneCondition::MultibeamCoverageActionDoneCondition(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{
  
}

BT::PortsList MultibeamCoverageActionDoneCondition::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<MultibeamCoverageActionClient> >("action_client", "{multibeam_coverage_action_client}", "Multibeam coverage action client")
  };
}

BT::NodeStatus MultibeamCoverageActionDoneCondition::tick()
{
  auto action_client_bb = getInput<std::shared_ptr<MultibeamCoverageActionClient> >("action_client");
  if(!action_client_bb)
  {
    throw BT::RuntimeError("MultibeamCoverageActionDoneCondition node named ",name(), " missing required input [action_client]: ", action_client_bb.error() );
  }
  if (action_client_bb.value()->done_)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}


}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::MultibeamCoverageActionCancel>("MultibeamCoverageActionCancel");
  factory.registerNodeType<project11_navigation::MultibeamCoverageActionDoneCondition>("MultibeamCoverageActionDoneCondition");
  factory.registerNodeType<project11_navigation::MultibeamCoverageActionSetGoal>("MultibeamCoverageActionSetGoal");
  factory.registerNodeType<project11_navigation::MultibeamCoverageActionUpdateTask>("MultibeamCoverageActionUpdateTask");
}
