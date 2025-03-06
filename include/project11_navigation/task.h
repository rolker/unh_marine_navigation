#ifndef PROJECT11_NAVIGATION_TASK_H
#define PROJECT11_NAVIGATION_TASK_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/time.hpp"
#include "project11_navigation/task_list.h"
#include "yaml-cpp/yaml.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace project11_navigation
{

class Task;
using TaskPtr = std::shared_ptr<Task>;
using TaskConstPtr = std::shared_ptr<const Task>;


// Represents the execution state of a task.
class Task
{
public:
  virtual ~Task() = default;

  /// Creates a new Task.
  static TaskPtr create(const project11_nav_msgs::msg::TaskInformation& task_msg, TaskPtr parent);

  static TaskPtr create(const project11_nav_msgs::msg::TaskInformation& task_msg, rclcpp::Clock::SharedPtr clock);


  /// Update or replace this task's TaskInformation.
  /// If check_id is true (default), only replaces the message
  /// if the new message has the same id as the existing message.
  /// Returns true if the message got replaced.
  bool update(const project11_nav_msgs::msg::TaskInformation& task_msg, bool check_id = true);

  /// Updates TaskInformation messages for this task
  /// and children tasks. Existing children tasks may be updated
  /// and new children tasks may be added. Existing children tasks
  /// that are not in task_msgs are discarded.
  void update(const std::vector<project11_nav_msgs::msg::TaskInformation>& task_msgs);

  /// Returns TaskInformation message associated with this task. 
  const project11_nav_msgs::msg::TaskInformation& message() const;

  /// Returns true if this task and optionally children tasks
  /// are all done. Children tasks are only checked if the
  /// recursive argument is true. Defaults to not checking
  /// children. 
  bool done(bool recursive=false) const;

  const TaskList& children() const;

  /// Sets the TaskInformation's done flag to true and updates
  /// last changed time.
  void setDone();

  /// Sets the id of the task argument as a child of this task.
  /// The id parameter gets added to this TaskInformation's id
  /// separated with a '/'.
  void setChildID(std::shared_ptr<Task> task, std::string id);

  /// Sets the TaskInformation id and updates the modification time.
  void setID(std::string id);

  /// Generates a child id by adding '/' and id to this task's id.  
  std::string getChildID(std::string id) const;


  /// Returns YAML decoded data from the message.
  YAML::Node data() const;

  /// Sets the data of the TaskInformation message and
  /// updates the modification time.
  void setData(const YAML::Node& data);

  /// Returns a data item from the parsed data member of the message
  /// optionally recursing up the parents.
  YAML::Node dataItem(std::string key, bool recurse_up=true) const;

  /// Returns YAML decoded data from the message.
  YAML::Node status() const;

  /// Sets the status of the TaskInformation message and
  /// updates the modification time.
  void setStatus(const YAML::Node &status);

  /// Returns time TaskInformation was last updated
  rclcpp::Time lastUpdateTime() const;

  /// Returns the first pose in this task's pose list. If
  /// this task's pose list is empty, optionally check
  /// the children recursively (depth first).
  bool getFirstPose(geometry_msgs::msg::PoseStamped& pose, bool recursive = false) const;

  /// Returns the last pose in this task's pose list. If
  /// this task's pose list is empty, optionally check
  /// the children recursively (depth first).
  bool getLastPose(geometry_msgs::msg::PoseStamped& pose, bool recursive = false) const;

  /// Creates a new task and inserts it as a child before task
  /// if not null. If task argument is null, the
  /// new task is appended to the children. No new task
  /// is created if task is not nullptr and is not found.
  std::shared_ptr<Task> createChildTaskBefore(std::shared_ptr<Task> task = {}, std::string type = "");


  /// Remove children tasks.
  void clearChildren();

  std::shared_ptr<Task> self();

private:
  // \todo make this private without breaking the create() method.
  Task(rclcpp::Clock::SharedPtr clock);

  std::weak_ptr<Task> self_;
  std::weak_ptr<Task> parent_task_;

  project11_nav_msgs::msg::TaskInformation message_;
  TaskList children_;
  rclcpp::Time last_update_time_;
  rclcpp::Clock::SharedPtr clock_;
};


} // namespace project11_navigation

#endif
