#ifndef PROJECT11_NAVIGATION_BAXEVANI_CONTROLLER_H
#define PROJECT11_NAVIGATION_BAXEVANI_CONTROLLER_H

#include <behaviortree_cpp/bt_factory.h>
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace project11_navigation
{

class BaxevaniController: public BT::StatefulActionNode
{
public:
  BaxevaniController(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  nav_msgs::msg::Odometry last_odom_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acceleration_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_acceleration_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr u_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr a_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr alpha_publisher_;

};

} // namespace path_follower

#endif
