#include <project11_navigation/occupancy_grid.h>

namespace project11_navigation
{

OccupancyGrid::OccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid)
  :grid_(grid)
{

}

const nav_msgs::msg::OccupancyGrid& OccupancyGrid::message() const
{
  return grid_;
}

int8_t OccupancyGrid::getValue(const geometry_msgs::msg::Point &point) const
{
  if(point.x >= grid_.info.origin.position.x && point.y >= grid_.info.origin.position.y)
  {
    uint32_t i = (point.x - grid_.info.origin.position.x)/grid_.info.resolution;
    uint32_t j = (point.y - grid_.info.origin.position.y)/grid_.info.resolution;
    if(i < grid_.info.width &&j < grid_.info.height)
    {
      return grid_.data[j*grid_.info.width+i];
    }
  }

  return -1;
}

} // namespace project11_navigation
