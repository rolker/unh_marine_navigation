#include "marine_nav_tasks/task.h"
#include "marine_nav_utilities/utilities.h"
#include "rclcpp/rclcpp.hpp"

namespace marine_nav_tasks
{

Task::Task(rclcpp::Clock::SharedPtr clock):
  children_(this),
  last_update_time_(clock->now()),
  clock_(clock)
{
}

std::shared_ptr<Task> Task::create(const TaskInformation& task_msg, TaskPtr parent_task)
{
  auto new_task = std::shared_ptr<Task>(new Task(parent_task->clock_));
  
  new_task->message_ = task_msg;
  new_task->self_ = new_task;
  new_task->parent_task_ = parent_task;
  return new_task;
}

std::shared_ptr<Task> Task::create(const TaskInformation& task_msg, rclcpp::Clock::SharedPtr clock)
{
  auto new_task = std::shared_ptr<Task>(new Task(clock));
  
  new_task->message_ = task_msg;
  new_task->self_ = new_task;
  return new_task;
}


std::shared_ptr<Task> Task::self()
{
  return self_.lock();
}

bool Task::update(const TaskInformation& task_msg, bool check_id)
{
  if(!check_id || message_.id == task_msg.id)
  {
    if(message_ != task_msg)
      last_update_time_ = clock_->now();
    message_= task_msg;
    return true;
  }
  return false;
}

void Task::update(const std::vector<TaskInformation>& task_msgs)
{
  for(auto task_msg: task_msgs)
    update(task_msg);
  children_.update(task_msgs);
}

const Task::TaskInformation & Task::message() const
{
  return message_;
}

void Task::setDone()
{
  message_.done = true;
  last_update_time_ = clock_->now();
}

YAML::Node Task::data() const
{
  return YAML::Load(message_.data);
}

void Task::setData(const YAML::Node& data)
{
  std::stringstream ss;
  ss << data;
  message_.data = ss.str();
  last_update_time_ = clock_->now();
}

YAML::Node Task::dataItem(std::string key, bool recurse_up) const
{
  auto d = data();
  if(d[key])
    return d[key];
  if(recurse_up && !parent_task_.expired())
    return parent_task_.lock()->dataItem(key, recurse_up);
  return {};
}

YAML::Node Task::status() const
{
  return YAML::Load(message_.status);
}

void Task::setStatus(const YAML::Node& status)
{
  std::stringstream ss;
  ss << status;
  message_.status = ss.str();
  last_update_time_ = clock_->now();
}

void Task::setID(std::string id)
{
  message_.id = id;
  last_update_time_ = clock_->now();
}

void Task::setChildID(std::shared_ptr<Task> task, std::string id)
{
  task->setID(message_.id + "/" + id);
}

bool Task::done(bool recursive) const
{
  if(recursive)
    return message_.done && children_.allDone(true);
  return message_.done;
}

const TaskList& Task::children() const
{
  return children_;
}

rclcpp::Time Task::lastUpdateTime() const
{
  return last_update_time_;  
}

bool Task::getFirstPose(geometry_msgs::msg::PoseStamped& pose, bool recursive) const
{
  if(!message_.poses.empty())
  {
    pose = message_.poses.front();
    return true;
  }
  if(recursive)
    return children_.getFirstPose(pose, recursive);
  return false;
}

bool Task::getLastPose(geometry_msgs::msg::PoseStamped& pose, bool recursive) const
{
  if(!message_.poses.empty())
  {
    pose = message_.poses.back();
    return true;
  }
  if(recursive)
    return children_.getLastPose(pose, recursive);
  return false;
}

std::shared_ptr<Task> Task::createChildTaskBefore(std::shared_ptr<Task> task, std::string type)
{
  return children_.createTaskBefore(task, type);
}

std::string Task::getChildID(std::string id) const
{
  return message_.id+"/"+id;
}

void Task::clearChildren()
{
  children_.clear();
}


}  // namespace marine_nav_tasks
