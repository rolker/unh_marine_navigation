#include <memory>

#include "marine_nav_ca_safety/ca_safety_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<marine_nav_ca_safety::CaSafetyNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
