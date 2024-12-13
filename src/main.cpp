
#include "rclcpp/rclcpp.hpp"
#include "project11_navigation/navigator.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<project11_navigation::Navigator>("navigator");

  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
