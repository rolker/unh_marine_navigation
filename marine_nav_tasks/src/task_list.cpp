#include "marine_nav_tasks/task_list.h"
#include "marine_nav_tasks/task.h"
#include "rclcpp/rclcpp.hpp"

namespace marine_nav_tasks
{

TaskList::TaskList(Task* parent):parent_task_(parent)
{

}

void TaskList::update(const std::vector<TaskInformation>& task_msgs, rclcpp::Clock::SharedPtr clock)
{
  // Create a new vector adding tasks in order they appear.
  // Existing tasks may be copied from existing vector or new ones created if needed.
  std::vector<std::shared_ptr<Task> > new_task_list;
  // First, the direct children or top level tasks
  for(const auto& task_msg: task_msgs)
  {
    auto id_parts = splitChildID(task_msg.id);
    // Top level or direct child id? Yes if we have a second part but not a first part.
    if(id_parts.first.empty() && !id_parts.second.empty())
    {
      std::shared_ptr<Task> task;
      for(auto existing_task: tasks_)
      {
        if(existing_task->message().id == task_msg.id)
        {
          task = existing_task;
          break;
        }
      }
      if(!task)
      {
        if(parent_task_)
        {
          TaskPtr parent = parent_task_->self();
          task = Task::create(task_msg, parent);
        }
        else
        {
          task = Task::create(task_msg, clock);
        }
      }
      task->update(task_msgs);
      new_task_list.push_back(task);
    }
  }
  tasks_ = new_task_list;
}

std::pair<std::string, std::string> TaskList::splitChildID(const std::string& task_id) const
{
  std::pair<std::string, std::string> ret;
  std::string parent_path;
  if(parent_task_)
    parent_path = parent_task_->message().id+"/";
  // If we have a parent, does the id begin with the parent's id? 
  if(!parent_task_ || task_id.substr(0, parent_path.size()) == parent_path)
  {
    std::string child_part = task_id;
    // remove the common beginning as well as the delimiter
    if(task_id.size() > parent_path.size())
      child_part = task_id.substr(parent_path.size());
    // find last delimiter, if any
    auto delimiter = child_part.rfind("/");
    if(delimiter == std::string::npos)
      ret.second = child_part;
    else
    {
      ret.first = child_part.substr(delimiter+1);
      ret.second = child_part.substr(0,delimiter);
    }
  }
  return ret;
}

const std::vector<std::shared_ptr<Task> >& TaskList::tasks() const
{
  return tasks_;
}

std::vector<TaskList::TaskInformation> TaskList::taskMessages() const
{
  std::vector<TaskInformation> ret;
  std::vector<TaskInformation> children_messages;
  for(auto t: tasks_)
  {
    ret.push_back(t->message());
    auto this_children_messages = t->children().taskMessages();
    children_messages.insert(children_messages.end(), this_children_messages.begin(), this_children_messages.end());
  }
  ret.insert(ret.end(), children_messages.begin(), children_messages.end());
  return ret;
}

std::vector<std::shared_ptr<Task> > TaskList::tasksByPriority(bool skip_done) const
{
  std::map<int, std::vector<std::shared_ptr<Task> > > priority_map;
  for(auto t: tasks_)
    if(!skip_done || !t->done())
      priority_map[t->message().priority].push_back(t);

  std::vector<std::shared_ptr<Task> > ret;
  for(auto task_list: priority_map)
    for(auto t: task_list.second)
      ret.push_back(t);

  return ret;
}

bool TaskList::getFirstPose(geometry_msgs::msg::PoseStamped& pose, bool recursive) const
{
  for(auto tp: tasks_)
    if(tp && tp->getFirstPose(pose, recursive))
      return true;
  return false;
}

bool TaskList::getLastPose(geometry_msgs::msg::PoseStamped& pose, bool recursive) const
{
  for(auto t = tasks_.rbegin(); t != tasks_.rend(); t++)
    if(*t && (*t)->getLastPose(pose, recursive))
      return true;
  return false;
}

std::shared_ptr<Task> TaskList::createTaskBefore(std::shared_ptr<Task> task, std::string type)
{
  auto task_iterator = tasks_.begin();
  while(task && task_iterator != tasks_.end() && *task_iterator != task)
    task_iterator++;
  
  std::shared_ptr<Task> ret;

  if(!task || task_iterator != tasks_.end())
  {
    // we found the target task
    TaskInformation tm;
    tm.type = type;
    if(task)
      tm.priority = task->message().priority;
    TaskPtr parent;
    if(parent_task_)
      parent = parent_task_->self();
    ret = Task::create(tm, parent);
    tasks_.insert(task_iterator, ret);
  }
  return ret;
}

std::string TaskList::generateUniqueID(const std::string& prefix, std::shared_ptr<Task> skip) const
{
  std::string path;
  if(parent_task_)
    path = parent_task_->message().id+"/";
  for(int i = 0; i < 1000; i++)
  {
    std::string candidate = path+prefix;
    if(i < 10)
      candidate += "00";
    else if(i<100)
      candidate += "0";
    candidate += std::to_string(i);
    bool exists = false;
    for(auto t: tasks_)
      if(t && t->message().id == candidate)
      {
        if(!skip || t!=skip)
        {
          exists = true;
          break;
        }
      }
    if(!exists)
      return candidate;
  }

  return "";
}

bool TaskList::allDone(bool recursive) const
{
  for(auto t: tasks_)
    if(!t->done(recursive))
      return false;
  return true;
}

void TaskList::clear()
{
  tasks_.clear();
}

}  // namespace marine_nav_tasks
