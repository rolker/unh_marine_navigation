#ifndef PROJECT11_NAVIGATION_ENVIRONMENT_H
#define PROJECT11_NAVIGATION_ENVIRONMENT_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "project11_nav_msgs/msg/robot_state.hpp"
#include "project11_navigation/occupancy_grid.h"

namespace project11_navigation
{

// Represent what is known about the environment from static data
// such as charts and perception data from sensors
class Environment
{
public:
  using Ptr = std::shared_ptr<Environment>;

  Environment(rclcpp_lifecycle::LifecycleNode::WeakPtr node);

  // Contains copies of grid maps for use where the maps shouldn't change
  // such as planners that expect a static map
  struct Snapshot
  {
    std::map<std::string, grid_map::GridMap> static_grids;
    std::map<double, std::vector<std::string> > static_grids_by_resolution;

    std::map<std::string, grid_map::GridMap> dynamic_grids;

    double getCost(const project11_nav_msgs::msg::RobotState& from_state, const project11_nav_msgs::msg::RobotState& to_state, double robot_comfort_radius);

  };

  // Generates and returns a snapshot.
  Snapshot snapshot(bool dynamic_only=false) const;

  std::string mapFrame() const;

  std::shared_ptr<OccupancyGrid> localCostmap() const;
  
private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::UniquePtr &data);

  struct Grid
  {
    void gridCallback(const grid_map_msgs::msg::GridMap::UniquePtr& data);
    void subscribe(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string topic);

    grid_map::GridMap grid_map;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber;
  };

  std::map<std::string, Grid> static_grids_;
  std::map<std::string, Grid> dynamic_grids_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_subscriber_;
  nav_msgs::msg::OccupancyGrid local_costmap_;
};


} // namespace project11_navigation


#endif
