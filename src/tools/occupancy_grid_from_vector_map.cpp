#include "rclcpp/rclcpp.hpp"

#include "project11_nav_msgs/msg/occupancy_vector_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "grid_map_ros/grid_map_ros.hpp"


class OccupancyGridFromVectorMap : public rclcpp::Node
{
public:
  OccupancyGridFromVectorMap()
  : Node("occupancy_grid_from_vector_map")
  {
    declare_parameter("resolution", resolution_);

    output_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("output", 1);

    auto input_subscription_ = this->create_subscription<project11_nav_msgs::msg::OccupancyVectorMap>("input", 1, std::bind(&OccupancyGridFromVectorMap::callback, this, std::placeholders::_1));
  }

private:
  void callback(const project11_nav_msgs::msg::OccupancyVectorMap::UniquePtr& message)
  {
    resolution_ = get_parameter("resolution").as_double();
    auto width = message->bounds.max_pt.x - message->bounds.min_pt.x;
    auto height = message->bounds.max_pt.y - message->bounds.min_pt.y;

    grid_map::Position center(message->bounds.min_pt.x+width/2.0, message->bounds.min_pt.y+height/2.0);

    grid_map::GridMap map;
    map.setGeometry(grid_map::Length(width, height), resolution_);
    map.setPosition(center);
    map.setFrameId(message->header.frame_id);
    rclcpp::Time time(message->header.stamp.sec, message->header.stamp.nanosec);
    map.setTimestamp( time.nanoseconds());
    map.add("occupancy", message->default_occupancy_probability);


    for(const auto& polygon: message->polygons)
    {
      for(int row = 0; row < map.getSize()[1]; row++)
      {
        double wy;
        wy = map.getPosition().y() - map.getLength().y()/2.0;
        wy += map.getResolution()*row;

        std::set<double> nodes;

        const auto& points = polygon.polygon.points;

        for(int i = 0; i < points.size(); i++)
        {
          auto p0 = points[i];
          auto p1 = points[0];
          if(i+1 < points.size())
            p1 = points[i+1];

          if(p0.y < wy && p1.y >= wy || p1.y < wy && p0.y >= wy)
          {
            nodes.insert(p0.x+(wy-p0.y)/(p1.y-p0.y)*(p1.x-p0.x));
          }
        }

        auto node = nodes.begin();
        while(node != nodes.end())
        {
          auto next_node = node;
          next_node++;
          if(next_node == nodes.end())
            break;

          if(*node > map.getPosition().x() + map.getLength().x()/2.0)
            break;

          if(*next_node > map.getPosition().x() - map.getLength().x()/2.0)
          {
            for(auto x = *node; x <= *next_node; x += map.getResolution())
            {
              grid_map::Index i;
              if(map.getIndex(grid_map::Position(x, wy),i))
              {
                auto existing_cost = map.at("occupancy", i);
                if(std::isnan(existing_cost) || polygon.occupancy_probability > existing_cost)
                  map.at("occupancy", i) = polygon.occupancy_probability;
              }
            }
          }
          node = next_node;
          node++;
        }
      }
    }

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "occupancy", 0.0, 100.0, occupancy_grid);
    output_publisher_->publish(occupancy_grid);


  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr output_publisher_;
  rclcpp::Subscription<project11_nav_msgs::msg::OccupancyVectorMap>::SharedPtr input_subscription_;

  double resolution_ = 1.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridFromVectorMap>());
  rclcpp::shutdown();
  return 0;
}


