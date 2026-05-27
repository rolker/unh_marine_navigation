#include <memory>

#include "marine_nav_utilities/costmap_window_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<marine_nav_utilities::CostmapWindowNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
