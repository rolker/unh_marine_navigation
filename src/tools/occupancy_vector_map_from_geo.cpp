#include "rclcpp/rclcpp.hpp"

#include "project11_nav_msgs/msg/geo_occupancy_vector_map.hpp"
#include "project11_nav_msgs/msg/occupancy_vector_map.hpp"

#include "project11/tf2_utils.h"


class OccupancyVectorMapFromGeo : public rclcpp::Node
{
public:
  OccupancyVectorMapFromGeo()
  : Node("occupancy_vector_map_from_geo")
  {
    output_publisher_ = create_publisher<project11_nav_msgs::msg::OccupancyVectorMap>("output", 1);

    input_subscription_ = create_subscription<project11_nav_msgs::msg::GeoOccupancyVectorMap>("input", 1, std::bind(&OccupancyVectorMapFromGeo::callback, this, std::placeholders::_1));

    declare_parameter("frame_id", frame_id_);
  }

private:

  void callback(const project11_nav_msgs::msg::GeoOccupancyVectorMap::UniquePtr& message)
  {
    frame_id_ = get_parameter("frame_id").as_string();

    if(!transformations_)
      transformations_ = std::make_shared<project11::Transformations>(this->shared_from_this());
  

    if(transformations_->canTransform(frame_id_, message->header.stamp))
    {
      project11_nav_msgs::msg::OccupancyVectorMap output;
      output.header.frame_id = frame_id_;
      output.header.stamp = message->header.stamp;

      output.default_occupancy_probability = message->default_occupancy_probability;

      output.bounds.min_pt = transformations_->wgs84_to_map(message->bounds.min_pt, frame_id_, message->header.stamp);
      output.bounds.max_pt = transformations_->wgs84_to_map(message->bounds.max_pt, frame_id_, message->header.stamp);

      for(const auto& geo_polygon: message->polygons)
      {
        project11_nav_msgs::msg::OccupancyPolygon polygon;
        polygon.occupancy_probability = geo_polygon.occupancy_probability;
        for(const auto& gp: geo_polygon.polygon.points)
        { 
          auto p = transformations_->wgs84_to_map(gp, frame_id_, message->header.stamp);
          geometry_msgs::msg::Point32 p32;
          p32.x = p.x;
          p32.y = p.y;
          p32.z = p.z;
          polygon.polygon.points.push_back(p32);
        }
        output.polygons.push_back(polygon);
      }

      output_publisher_->publish(output);
    }
  }

  rclcpp::Publisher<project11_nav_msgs::msg::OccupancyVectorMap>::SharedPtr output_publisher_;
  rclcpp::Subscription<project11_nav_msgs::msg::GeoOccupancyVectorMap>::SharedPtr input_subscription_;

  std::shared_ptr<project11::Transformations> transformations_;
  std::string frame_id_ = "map";
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyVectorMapFromGeo>());
  rclcpp::shutdown();
  return 0;
}


