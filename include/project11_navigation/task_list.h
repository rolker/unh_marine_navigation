#ifndef PROJECT11_NAVIGATION_TASKLIST_H
#define PROJECT11_NAVIGATION_TASKLIST_H

#include "project11_nav_msgs/msg/task_information.hpp"
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

class Task;
using TaskPtr = std::shared_ptr<Task>;
using TaskConstPtr = std::shared_ptr<const Task>;

class TaskList;
using TaskListPtr = std::shared_ptr<TaskList>;
using TaskListConstPtr = std::shared_ptr<const TaskList>;

class TaskList
{
public:
  TaskList(Task* parent = nullptr);

  /// Updates or add tasks from task_msgs. Removes existing tasks
  /// that are not in task_msgs. Tasks are reordered if necessary
  /// to reflect the order in task_msgs.
  void update(const std::vector<project11_nav_msgs::msg::TaskInformation>& task_msgs, rclcpp::Node::SharedPtr node = {});

  /// Return list of direct children tasks. 
  const std::vector<std::shared_ptr<Task> >& tasks() const;

  /// Returns a list of TaskInformation messages, recursing 
  /// down the children. 
  std::vector<project11_nav_msgs::msg::TaskInformation> taskMessages() const;

  /// Returns a list of direct children Tasks sorted by priority. 
  std::vector<std::shared_ptr<Task> > tasksByPriority(bool skip_done = false) const;

  /// Returns the first pose among direct children,
  /// optionally recursively searching (depth first).
  bool getFirstPose(geometry_msgs::msg::PoseStamped& pose, bool recursive= false) const;

  /// Returns the last pose among direct children,
  /// optionally recursively searching (depth first).
  bool getLastPose(geometry_msgs::msg::PoseStamped& pose, bool recursive= false) const;

  /// Creates a new task and optionally inserts it before task.
  /// If task is not null and it is not found, no new task is created.
  /// Returns the new task or nullptr.
  std::shared_ptr<Task> createTaskBefore(std::shared_ptr<Task> task, std::string type = "");
  
  /// Returns true if all tasks in the list are done,
  /// optionally recursing down the children.
  bool allDone(bool recursive=false) const;

  /// Generates a unique id for a child task using
  /// the parent id (if not None) and prefix with a number
  /// appended. The first number from 0 to 999 resulting
  /// in a unique id is used. Empty string is returned if
  /// a unique id can't be found. If a skip task is supplied
  /// allow it's id to be reused.
  std::string generateUniqueID(const std::string &prefix, std::shared_ptr<Task> skip = {}) const; 

  /// Clears the tasks
  void clear();
private:
  /// Split the id of a potential child task into the
  /// relative path and direct child part.
  /// Returns empty strings if the id is not for a child task.
  /// The first component will be the relative path if not
  /// a direct children and the second component will be 
  /// the last component of the task id.
  std::pair<std::string, std::string> splitChildID(const std::string& task_id) const;

  std::vector<std::shared_ptr<Task> > tasks_;
  Task* parent_task_ = nullptr;
};

} // namespace project11_navigation

#endif
