#include "marine_nav_behavior_tree/plugins/action/set_polygon_from_task.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <marine_nav_tasks/task.h>

namespace marine_nav_behavior_tree
{

using TaskPtr = std::shared_ptr<marine_nav_tasks::Task>;

SetPolygonFromTask::SetPolygonFromTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList SetPolygonFromTask::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task", "{current_task}", "Task to get polygon from"),
    BT::InputPort<int>("start_index", "0", "Index of the first pose describing the polygon"),
    BT::InputPort<int>("end_index", "-1", "Index of the last pose in the polygon"),
    BT::OutputPort<geometry_msgs::msg::PolygonStamped>("polygon", "Polygon to set"),
  };
}

BT::NodeStatus SetPolygonFromTask::tick()
{
  auto task_bb = getInput<TaskPtr>("task");
  if(!task_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [task]: ", task_bb.error() );
  }

  auto task = task_bb.value();

  auto start_index_bb = getInput<int>("start_index");
  if(!start_index_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [start_index]: ", start_index_bb.error() );
  }
  auto end_index_bb = getInput<int>("end_index");
  if(!end_index_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [end_index]: ", end_index_bb.error() );
  }

  auto start_index = start_index_bb.value();
  auto end_index = end_index_bb.value();

  if(end_index < 0)
  {
    end_index = task->message().poses.size() + end_index;
  }

  const auto& poses = task->message().poses;

  geometry_msgs::msg::PolygonStamped polygon;
  
  for(std::size_t i = start_index; i <= end_index && i < poses.size(); i++)
  {
    if(i == start_index)
    {
      polygon.header = poses[i].header;
    }
    geometry_msgs::msg::Point32 pt;
    pt.x = poses[i].pose.position.x;
    pt.y = poses[i].pose.position.y;
    pt.z = poses[i].pose.position.z;
    polygon.polygon.points.push_back(pt);
  }

  setOutput("polygon", polygon);

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree
