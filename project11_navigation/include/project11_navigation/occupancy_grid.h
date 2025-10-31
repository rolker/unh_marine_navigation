#ifndef PROJECT11_NAVIGATION_OCCUPANCY_GRID_H
#define PROJECT11_NAVIGATION_OCCUPANCY_GRID_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace project11_navigation
{

class OccupancyGrid
{
public:
  OccupancyGrid(const nav_msgs::msg::OccupancyGrid &grid);

  const nav_msgs::msg::OccupancyGrid& message() const;
  int8_t getValue(const geometry_msgs::msg::Point &point) const;
private:
  nav_msgs::msg::OccupancyGrid grid_;

};

} // namespace project11

#endif
