#include "project11_navigation/context.h"
#include "project11_navigation/plugins/action/robot_capabilities_loader.h"
#include <project11_navigation/robot_capabilities.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

RobotCapabilitiesLoader::RobotCapabilitiesLoader(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
}

BT::PortsList RobotCapabilitiesLoader::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<Context> >("context", "{context}", "Navigation context"),
    BT::OutputPort<double>("turn_radius", "{robot_turn_radius}", "Default turn radius"),
    BT::OutputPort<geometry_msgs::msg::Twist>("maximum_velocity", "{robot_maximum_velocity}", ""),
    BT::OutputPort<geometry_msgs::msg::Twist>("minimum_velocity", "{robot_minimum_velocity}", ""),
    BT::OutputPort<geometry_msgs::msg::Twist>("default_velocity", "{robot_default_velocity}", "Default cruise speed"),
    BT::OutputPort<geometry_msgs::msg::Accel>("maximum_acceleration", "{robot_maximum_acceleration}", "Maximum available acceleration"),
    BT::OutputPort<geometry_msgs::msg::Accel>("default_acceleration", "{robot_default_acceleration}", "Default acceleration that should be used for normal operations"),
    BT::OutputPort<geometry_msgs::msg::Accel>("maximum_deceleration", "{robot_maximum_deceleration}", "Powered deceleration"),
    BT::OutputPort<geometry_msgs::msg::Accel>("default_deceleration", "{robot_default_deceleration}", "Drifting deceleration"),
    BT::OutputPort<geometry_msgs::msg::Polygon>("footprint", "{robot_footprint}", ""),
    BT::OutputPort<double>("radius", "{robot_radius}", ""),

    BT::OutputPort<double>("default_speed", "{robot_default_speed}", ""),
    BT::OutputPort<double>("stopping_time", "{robot_stopping_time}", ""),
    BT::OutputPort<double>("stopping_distance", "{robot_stopping_distance}", ""),
  };
}

BT::NodeStatus RobotCapabilitiesLoader::tick()
{
  auto context = getInput<std::shared_ptr<Context> >("context");
  if(!context)
  {
    throw BT::RuntimeError(name(), " missing required input [path]: ", context.error() );
  }

  const auto& rc = context.value()->robot_capabilities();
  setOutput("turn_radius", rc.getTurnRadiusAtSpeed(rc.default_velocity.linear.x));
  setOutput("maximum_velocity", rc.max_velocity);
  setOutput("minimum_velocity", rc.min_velocity);
  setOutput("default_velocity", rc.default_velocity);
  setOutput("maximum_acceleration", rc.max_acceleration);
  setOutput("default_acceleration", rc.default_acceleration);
  setOutput("maximum_deceleration", rc.max_deceleration);
  setOutput("default_deceleration", rc.default_deceleration);
  setOutput("footprint", rc.footprint);
  setOutput("radius", rc.radius);

  setOutput("default_speed", rc.default_velocity.linear.x);

  // how long to stop from max speed
  if(rc.default_deceleration.linear.x < 0.0)
  {
    auto stopping_time = -rc.max_velocity.linear.x/rc.default_deceleration.linear.x;
    auto stopping_distance = stopping_time*rc.max_velocity.linear.x/2.0;

    setOutput("stopping_time", stopping_time);
    setOutput("stopping_distance", stopping_distance);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::RobotCapabilitiesLoader>("RobotCapabilitiesLoader");
}
