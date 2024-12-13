#include "project11_navigation/environment.h"

namespace project11_navigation
{

Environment::Environment(rclcpp_lifecycle::LifecycleNode::WeakPtr node_ptr)
{
  auto node = node_ptr.lock();
  std::vector<std::string> static_grid_topics;
  node->declare_parameter("grids/static", static_grid_topics);
  node->get_parameter("grids/static", static_grid_topics);
  for(auto sg: static_grid_topics)
  {
    static_grids_[sg].subscribe(node, sg);
  }

  local_costmap_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>("local_costmap", 1, std::bind(&Environment::occupancyGridCallback, this, std::placeholders::_1));
}

Environment::Snapshot Environment::snapshot(bool dynamic_only) const
{
  Snapshot ret;
  if(!dynamic_only)
  {
    for(auto& g: static_grids_)
      ret.static_grids[g.first] = g.second.grid_map;
    for(auto& sg: ret.static_grids)
      ret.static_grids_by_resolution[sg.second.getResolution()].push_back(sg.first);
  }
  for(auto& g: dynamic_grids_)
    ret.dynamic_grids[g.first] = g.second.grid_map;
  return ret;
}

std::string Environment::mapFrame() const
{
  if(!static_grids_.empty())
    return static_grids_.begin()->second.grid_map.getFrameId();
  if(!dynamic_grids_.empty())
    return dynamic_grids_.begin()->second.grid_map.getFrameId();
  return "";
}

void Environment::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::UniquePtr &data)
{
  local_costmap_ = *data;
}

std::shared_ptr<OccupancyGrid> Environment::localCostmap() const
{
  return std::make_shared<OccupancyGrid>(local_costmap_);
}


void Environment::Grid::subscribe(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string topic)
{
  auto callback = [this](const grid_map_msgs::msg::GridMap::UniquePtr& msg)
  {
    this->gridCallback(msg);
  };
  subscriber = node->create_subscription<grid_map_msgs::msg::GridMap> (topic, 1, callback);
}

void Environment::Grid::gridCallback(const grid_map_msgs::msg::GridMap::UniquePtr& data)
{
  grid_map::GridMapRosConverter::fromMessage(*data, grid_map);
}

double Environment::Snapshot::getCost(const project11_nav_msgs::msg::RobotState& from_state, const project11_nav_msgs::msg::RobotState& to_state, double robot_comfort_radius)
{
  grid_map::Position position(to_state.pose.position.x, to_state.pose.position.y);
  float dynamic_weight = 1.0;
  for(auto grid: dynamic_grids)
  {
    grid_map::Index index;
    if(grid.second.getIndex(position, index))
    {
      float intensity = grid.second.at("intensity", index);
      for(grid_map::CircleIterator i(grid.second, position, robot_comfort_radius); !i.isPastEnd(); ++i)
        intensity = std::max(intensity, grid.second.at("intensity", *i));
      dynamic_weight = 1.0-intensity;
    }
  }
  for(auto gv: static_grids_by_resolution)
    for(auto grid_label: gv.second)
    {
      grid_map::Index index;
      grid_map::GridMap& grid = static_grids[grid_label];
      if(grid.getIndex(position, index))
      {
        float ret = grid.at("speed", index);
        for(grid_map::CircleIterator i(grid, position, robot_comfort_radius); !i.isPastEnd(); ++i)
          ret = std::min(ret, grid.at("speed", *i));
        return ret*dynamic_weight;
      }
    }
  return -1.0;

}


} // namespace project11_navigation
