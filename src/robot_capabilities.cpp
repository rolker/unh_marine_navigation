#include "project11_navigation/robot_capabilities.h"
#include "project11_navigation/utilities.h"
#include "rclcpp/rclcpp.hpp"

namespace project11_navigation
{

RobotCapabilities::RobotCapabilities(rclcpp::Node::SharedPtr node)
{
  // XmlRpc::XmlRpcValue value;
  // if(nh.getParam("robot/turn_radius", value))
  // {
  //   if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
  //     for(int i = 0; i < value.size(); ++i)
  //       turn_radius_map[static_cast<double>(value[i]["velocity"])] = static_cast<double>(value[i]["radius"]);
  //   else
  //     nh.getParam("robot/turn_radius", turn_radius_map[0.0]);
  // }
  node->declare_parameter("robot/turn_radius", turn_radius);

  declareLinearAngularParameters(node, "robot/max_velocity", max_velocity, max_velocity);
  declareLinearAngularParameters(node, "robot/min_velocity", min_velocity, min_velocity);
  declareLinearAngularParameters(node, "robot/default_velocity", default_velocity, default_velocity);

  declareLinearAngularParameters(node, "robot/max_acceleration", max_acceleration, max_acceleration);
  declareLinearAngularParameters(node, "robot/default_acceleration", default_acceleration, default_acceleration);

  declareLinearAngularParameters(node, "robot/max_deceleration", max_deceleration, max_deceleration);
  declareLinearAngularParameters(node, "robot/default_deceleration", default_deceleration, default_deceleration);

  std::vector<double> footprint_vector;
  node->declare_parameter("robot/footprint", footprint_vector);
  for(size_t i = 0; i < footprint_vector.size()-1; i += 2)
  {
    geometry_msgs::msg::Point32 p;
    p.x = footprint_vector[i];
    p.y = footprint_vector[i+1];
    footprint.points.push_back(p);
  }
  for(auto p: footprint.points)
    radius = std::max(radius, sqrt(p.x*p.x + p.y*p.y));

  node->declare_parameter("robot/radius", radius);
  node->get_parameter("robot/radius", radius);

}

double RobotCapabilities::getTurnRadiusAtSpeed(double speed) const
{
  if(!turn_radius_map.empty())
  {
    if(turn_radius_map.size() == 1)
      return turn_radius_map.begin()->second;
    auto low = turn_radius_map.begin();
    if(speed <= low->first)
      return low->second;
    auto high = low;
    ++high;
    while (high != turn_radius_map.end() && high->first < speed)
    {
      low = high;
      ++high;
    }
    if(high == turn_radius_map.end())
      return low->second;
    if(high->first <= speed)
      return high->second;
    return low->second + ((speed-low->first)/(high->first-low->first))*(high->second-low->second);
  }
  return 0.0;
}



}