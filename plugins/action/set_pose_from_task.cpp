#include <project11_navigation/plugins/action/set_pose_from_task.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <project11_navigation/task.h>

namespace project11_navigation
{

SetPoseFromTask::SetPoseFromTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList SetPoseFromTask::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task"),
    BT::InputPort<int>("pose_index"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
  };
}

BT::NodeStatus SetPoseFromTask::tick()
{
  auto task = getInput<TaskPtr>("task");
  if(!task)
  {
    throw BT::RuntimeError("missing required input [task]: ", task.error() );
  }

  auto pose_index = getInput<int>("pose_index");
  if(!pose_index)
  {
    throw BT::RuntimeError("missing required input [pose_index]: ", pose_index.error() );
  }

  int index = pose_index.value();
  if(index < 0) // python style count from the end
    index = task.value()->message().poses.size()+index;
  if(index < 0 || index >= task.value()->message().poses.size())
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "SetPoseFromTask node named " << name() << " index " << pose_index.value() << " out of range for task with " << task.value()->message().poses.size() << " poses");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("pose", task.value()->message().poses[pose_index.value()]);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::SetPoseFromTask>("SetPoseFromTask");
}
