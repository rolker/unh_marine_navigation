#include "project11_navigation/navigator.h"
#include "rclcpp/rclcpp.hpp"

#include "project11_navigation/context.h"
#include "project11_navigation/utilities.h"
#include "project11_navigation/task_list.h"
#include <filesystem>
#include "project11_navigation/bt_types.h"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/utils/shared_library.h"
#include "nav2_util/string_utils.hpp"
#include "plugins_list.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace project11_navigation
{

Navigator::Navigator(const std::string & node_name):
  rclcpp_lifecycle::LifecycleNode(node_name)
{

  std::vector<std::string> plugin_lib_names;
  plugin_lib_names = nav2_util::split(project11_navigation::details::BT_BUILTIN_PLUGINS, ';');
  plugin_lib_names.push_back("ccom_planner_bt_plugin");
  BT::SharedLibrary loader;
  for (const auto & p : plugin_lib_names) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

CallbackReturn Navigator::on_configure(const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::LifecycleNode::on_configure(state);
  registerJsonDefinitions();

  display_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization_markers", 10);

  rclcpp::QoS qos(1);
  qos.transient_local();

  navigation_state_publisher_ = create_publisher<std_msgs::msg::String>("~/navigation_state", qos);

  auto node = shared_from_this();
  context_ = std::make_shared<Context>(node);

  declare_parameter("log_file", std::string());
  declare_parameter("debug", false);

  auto action_name = std::string(get_name())+"/run_tasks";

  // begin from navigation2/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp
  // Name client node after action name
  std::string client_node_name = action_name;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r",
      std::string("__node:=") +
      std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
      "-p",
      "use_sim_time:=" +
      std::string(node->get_parameter("use_sim_time").as_bool() ? "true" : "false"),
      "--"});

  // Support for handling the topic-based goal pose from rviz
  auto client_node = std::make_shared<rclcpp::Node>("_", options);

  rclcpp::copy_all_parameter_values(this, client_node);
  // end from nav2...

  blackboard_ = BT::Blackboard::create();
  blackboard_->set("context", context_);
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Navigator::on_activate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_activate(state);

  tree_ = buildBehaviorTree();
    
  groot_ = std::make_shared<BT::Groot2Publisher>(tree_);

  std::string log_file;
  get_parameter("log_file", log_file);
  
  if(!log_file.empty())
    logger_ = std::make_shared<BT::FileLogger2>(tree_, log_file);

  bool debug = false;
  get_parameter("debug", debug);
  if(debug)
    console_logger_ = std::make_shared<BT::StdCoutLogger>(tree_);

  auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunTasks::Goal> goal)
  {
    return this->handleGoal(uuid, goal);
  };

  auto handle_cancel = [this](const std::shared_ptr<GoalHandleRunTasks> goal)
  {
    return this->handleCancel(goal);
  };

  auto handle_accepted = [this](const std::shared_ptr<GoalHandleRunTasks> goal)
  {
    this->handleAccepted(goal);
  };

  auto action_name = std::string(get_name())+"/run_tasks";
  action_server_ = rclcpp_action::create_server<RunTasks>(
    shared_from_this(),
    action_name,
    handle_goal,
    handle_cancel,
    handle_accepted
  );

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](const nav_msgs::msg::Odometry::UniquePtr &odom)
  {
    this->odometryCallback(odom);
  });

  return CallbackReturn::SUCCESS;
}

CallbackReturn Navigator::on_deactivate(const rclcpp_lifecycle::State& state)
{
  rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

  odom_sub_.reset();
  action_server_.reset();
  console_logger_.reset();
  logger_.reset();
  groot_.reset();
  tree_.haltTree();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Navigator::on_cleanup(const rclcpp_lifecycle::State& state)
{
  rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

  blackboard_.reset();
  context_.reset();
  navigation_state_publisher_.reset();
  display_pub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Navigator::on_shutdown(const rclcpp_lifecycle::State& state)
{

  rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
  blackboard_.reset();
  context_.reset();
  navigation_state_publisher_.reset();
  display_pub_.reset();

  return CallbackReturn::SUCCESS;
}

BT::Tree Navigator::buildBehaviorTree()
{
  std::string bt_file =  ament_index_cpp::get_package_share_directory("project11_navigation")+"/behavior_trees/navigator.xml";
  factory_.registerBehaviorTreeFromFile(bt_file);

  //factory.registerFromROSPlugins();

  /// \todo Allow extra directories from param server

  std::vector<std::string> packages = {"project11_navigation"};

  for (auto package: packages)
  {
    // std::string tree_dir = ros::package::getPath(package)+"/behavior_trees";
    // for (auto const& entry : std::filesystem::directory_iterator(tree_dir)) 
    // {
    //   if( entry.path().extension() == ".xml")
    //   {
    //     factory.registerBehaviorTreeFromFile(entry.path().string());
    //   }
    // }
  }

  return factory_.createTree("NavigatorSequence", blackboard_);
}

rclcpp_action::GoalResponse Navigator::handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunTasks::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void Navigator::handleAccepted(const std::shared_ptr<GoalHandleRunTasks> goal_handle)
{
  if(goal_handle_ && goal_handle_->is_active())
  {
    goal_handle_->abort(generateResult());
    clearGoal();
  }
  goal_handle_ = goal_handle;
  const auto goal = goal_handle->get_goal();

  if(goal->tasks.empty())
    blackboard_->set("task_messages", std::shared_ptr<std::vector<project11_nav_msgs::msg::TaskInformation> >());
  else
    blackboard_->set("task_messages", std::make_shared<std::vector<project11_nav_msgs::msg::TaskInformation> >(goal->tasks));
}

rclcpp_action::CancelResponse Navigator::handleCancel(const std::shared_ptr<GoalHandleRunTasks> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

std::shared_ptr<Navigator::RunTasks::Result> Navigator::generateResult()
{
  auto result = std::make_shared<RunTasks::Result>();
  auto task_list_entry = blackboard_->getEntry("task_list");
  if(task_list_entry)
  {
    auto task_list = task_list_entry->value.cast<std::shared_ptr<TaskList> >();
    if(task_list)
      result->tasks = task_list->taskMessages();
  }
  return result;
}

void Navigator::clearGoal()
{
  blackboard_->set("task_messages", std::shared_ptr<std::vector<project11_nav_msgs::msg::TaskInformation> >());
  tree_.haltTree();
  updateNavigationState("inactive");
  goal_handle_.reset();
}

void Navigator::updateNavigationState(std::string nav_state)
{
  if(nav_state != last_navigation_state_)
  {
    std_msgs::msg::String nav_state_msg;
    nav_state_msg.data = nav_state;
    navigation_state_publisher_->publish(nav_state_msg);
    last_navigation_state_ = nav_state;
  }
}

void Navigator::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr& msg)
{
  context_->robot().odometryCallback(msg);
  if(goal_handle_)
  {
    if(goal_handle_->is_canceling())
    {
      goal_handle_->canceled(generateResult());
      clearGoal();
    }
    else
    {
      auto status = tree_.tickOnce();
      switch(status)
      {
        case BT::NodeStatus::SUCCESS:
        {
          goal_handle_->succeed(generateResult());
          clearGoal();
          break;
        }
        case BT::NodeStatus::FAILURE:
          goal_handle_->abort(generateResult());
          clearGoal();
          break;
        case BT::NodeStatus::RUNNING:
        {
          auto cmd_vel = blackboard_->getEntry("command_velocity");
          if(cmd_vel && cmd_vel->value.isType<geometry_msgs::msg::TwistStamped>())
          {
            context_->robot().sendControls(cmd_vel->value.cast<geometry_msgs::msg::TwistStamped>());
          }

          auto marker_array = blackboard_->getEntry("marker_array");
          if(marker_array && marker_array->value.isType<std::shared_ptr<visualization_msgs::msg::MarkerArray> >())
          {
            auto ma = marker_array->value.cast<std::shared_ptr<visualization_msgs::msg::MarkerArray> >();
            if(ma)
              display_pub_->publish(*ma);
          }

          auto nav_state = blackboard_->getEntry("navigation_state");
          if(nav_state && nav_state->value.isType<std::string>())
          {
            auto new_nav_state = nav_state->value.cast<std::string>();
            updateNavigationState(new_nav_state);
          }
        }
        break;
      }

      auto feedback = std::make_shared<RunTasks::Feedback>();
      auto current_nav_task = blackboard_->getEntry("active_task_id");
      if(current_nav_task && current_nav_task->value.isString())
        feedback->feedback.current_navigation_task = current_nav_task->value.cast<std::string>();
      auto task_list_entry = blackboard_->getEntry("task_list");
      if(task_list_entry)
      {
        auto task_list = task_list_entry->value.cast<std::shared_ptr<TaskList> >();
        if(task_list)
          feedback->feedback.tasks = task_list->taskMessages();
      }
      if(goal_handle_)
        goal_handle_->publish_feedback(feedback);
    }
  }

}


} // namespace project11_navigation
