#include <project11_navigation/plugins/action/baxevani_controller.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "project11/utils.h"
#include "project11_navigation/context.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"

namespace project11_navigation
{

BaxevaniController::BaxevaniController(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{


}

BT::PortsList BaxevaniController::providedPorts()
{
  return {
    BT::InputPort<Context::Ptr>("context", "{@context}", "Navigation context"),
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_path", "{navigation_path}", "Path to follow"),
    BT::InputPort<int>("current_navigation_segment", "{current_navigation_segment}", "Current segment of the path to follow"),
    BT::InputPort<double>("target_speed", "{target_speed}", "Target speed"),
    BT::InputPort<double>("ricatti_parameter", "1.8752285", "Ricatti parameter"),
    BT::InputPort<double>("px_gain", "100.78", "Proportional gain for the x axis"),
    BT::InputPort<double>("pw_gain", "20.055", "Proportional gain for the w axis"),
    BT::InputPort<double>("delta", "0.1", "Distance of the (sonar) sensor from the center of mass/position of imu"),
    BT::InputPort<double>("maximum_dt", "0.5", "Maximum time step"),
    BT::OutputPort<geometry_msgs::msg::TwistStamped>("command_velocity", "{command_velocity}", "Output commanded velocity"),
  };
}

BT::NodeStatus BaxevaniController::onStart()
{
  if(!acceleration_publisher_)
  {
    auto context = getInput<Context::Ptr>("context");
    if(!context)
    {
      throw BT::RuntimeError(name(), " missing required input [context]: ", context.error() );
    }
    auto node = context.value()->node().lock();

    acceleration_publisher_ = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("baxevani/debug/acceleration", 1);
    angular_acceleration_publisher_ = node->create_publisher<std_msgs::msg::Float32>("baxevani/debug/angular_acceleration", 1);
    u_publisher_ = node->create_publisher<std_msgs::msg::Float32>("baxevani/debug/u", 1);
    a_publisher_ = node->create_publisher<std_msgs::msg::Float32>("baxevani/debug/a", 1);
    alpha_publisher_ = node->create_publisher<std_msgs::msg::Float32>("baxevani/debug/alpha", 1);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BaxevaniController::onRunning()
{
  auto context_bb = getInput<Context::Ptr>("context");
  if(!context_bb)
  {
    throw BT::RuntimeError("missing required input [context]: ", context_bb.error() );
  }
  auto context = context_bb.value();

  auto navigation_path_bb = getInput<std::shared_ptr<std::vector< geometry_msgs::msg::PoseStamped> > >("navigation_path");
  if(!navigation_path_bb)
  {
    throw BT::RuntimeError("missing required input [navigation_path]: ", navigation_path_bb.error() );
  }
  auto navigation_path = navigation_path_bb.value();

  int segment_count = std::max<int>(0,navigation_path->size()-1);

  auto current_segment = getInput<int>("current_navigation_segment");
  if(!current_segment)
  {
    throw BT::RuntimeError("missing required input [current_navigation_segment]: ", current_segment.error() );
  }

  if(current_segment.value() < 0 || current_segment.value() > segment_count)
    return BT::NodeStatus::FAILURE;

  // We're done if we are at the segment past the last one
  if(current_segment.value() == segment_count)
    return BT::NodeStatus::SUCCESS;

  auto odom = context->robot().odometry();

  auto tf_buffer = context->tfBuffer();
  
  double target_speed = 0.0;
  auto target_speed_bb = getInput<double>("target_speed");
  if(!target_speed_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [target_speed]: ", target_speed_bb.error() );
  }
  target_speed = target_speed_bb.value();

  auto ricatti_parameter = getInput<double>("ricatti_parameter");
  if(!ricatti_parameter)
  {
    throw BT::RuntimeError(name(), " missing required input [ricatti_parameter]: ", ricatti_parameter.error() );
  }

  auto px_gain_bb = getInput<double>("px_gain");
  if(!px_gain_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [px_gain]: ", px_gain_bb.error() );
  }

  auto pw_gain_bb = getInput<double>("pw_gain");
  if(!pw_gain_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [pw_gain]: ", pw_gain_bb.error() );
  }

  auto delta_bb = getInput<double>("delta");
  if(!delta_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [delta]: ", delta_bb.error() );
  }

  auto maximum_dt = getInput<double>("maximum_dt");
  if(!maximum_dt)
  {
    throw BT::RuntimeError(name(), " missing required input [maximum_dt]: ", maximum_dt.error() );
  }

  geometry_msgs::msg::TransformStamped base_to_map;
  try
  {
    base_to_map = tf_buffer->lookupTransform(navigation_path->front().header.frame_id , odom.child_frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    auto node = context->node().lock();
    RCLCPP_WARN_STREAM(node->get_logger(), "FollowPathCommand node named " << name() << " Error getting path to base_frame transform: " << ex.what());
    return BT::NodeStatus::FAILURE;
  }

  auto p1 = (*navigation_path)[current_segment.value()];
  auto p2 = (*navigation_path)[current_segment.value()+1];

  auto segment_dx = p2.pose.position.x - p1.pose.position.x;
  auto segment_dy = p2.pose.position.y - p1.pose.position.y;

  project11::AngleRadians segment_azimuth(atan2(segment_dy, segment_dx));
  auto segment_distance = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

  // vehicle distance and azimuth relative to the segment's start point
  double dx = p1.pose.position.x - base_to_map.transform.translation.x;
  double dy = p1.pose.position.y - base_to_map.transform.translation.y;
  auto vehicle_distance = sqrt(dx*dx+dy*dy);

  project11::AngleRadians vehicle_azimuth(atan2(-dy, -dx));

  auto error_azimuth = vehicle_azimuth - segment_azimuth;
     
  auto sin_error_azimuth = sin(error_azimuth);
  auto cos_error_azimuth = cos(error_azimuth);

  // Distance traveled along the line.
  auto progress = vehicle_distance*cos_error_azimuth;

  auto cross_track_error = vehicle_distance*sin_error_azimuth;

  
  project11::AngleRadians heading(tf2::getYaw(base_to_map.transform.rotation));

  project11::AngleRadians target_heading = segment_azimuth;

  project11::AngleRadians theta(project11::AngleRadiansZeroCentered(target_heading - heading));

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = odom.header.stamp;
  cmd_vel.header.frame_id = odom.child_frame_id;

  auto dt = (rclcpp::Time(odom.header.stamp) - rclcpp::Time(last_odom_.header.stamp)).seconds();
  if(dt <= maximum_dt.value() && dt > 0.0)
  {
    auto twist = odom.twist.twist;

    double sensor_velx = twist.linear.x;
    double sensor_vely = twist.linear.y;
    double sensor_velw = twist.angular.z;

    auto last_twist = last_odom_.twist.twist;

    double prev_sensor_velx = last_twist.linear.x;
    double prev_sensor_vely = last_twist.linear.y;
    double prev_sensor_velw = last_twist.angular.z;

    double current_y = cross_track_error;

    double r = ricatti_parameter.value();
    double px_gain = px_gain_bb.value();
    double pw_gain = pw_gain_bb.value();
    
    double v_d = target_speed;
    double delta = delta_bb.value();

    double acc_x = (sensor_velx - prev_sensor_velx) / dt; //Acceleration on the surge axis
    double acc_y = (sensor_vely - prev_sensor_vely) / dt; //Acceleration on the sway axis
    double acc_w = (sensor_velw - prev_sensor_velw) / dt; //Acceleration on the sway axis

    geometry_msgs::msg::Vector3Stamped acceleration_msg;
    acceleration_msg.header = odom.header;
    acceleration_msg.header.frame_id = odom.child_frame_id;
    acceleration_msg.vector.x = acc_x;
    acceleration_msg.vector.y = acc_y;

    acceleration_publisher_->publish(acceleration_msg);

    std_msgs::msg::Float32 angular_acceleration_msg;
    angular_acceleration_msg.data = acc_w;

    angular_acceleration_publisher_->publish(angular_acceleration_msg);
    

    double u = -(current_y + delta * sin(theta)) / sqrt(r) -
        (sensor_velx * sin(-theta) + (sensor_vely + delta * sensor_velw)
         * cos(-theta)) * sqrt(2.0) / pow(r, (1.0 / 4.0)); // Optimal feedback law for the system

    double a = 1 / r * (v_d - sensor_velx) * cos(theta) - u * sin(theta) +
        delta * pow(sensor_velw, 2.0) + (pow(sin(heading), 2.0) -
        pow(cos(heading), 2.0)) * sensor_vely * sensor_velw; // Linear acceleration on the x axis (axis of motion)

    double alpha = 1 / (r * delta) * (v_d - sensor_velx) * sin(theta) + u * 
        cos(theta) / delta - sensor_velx * sensor_velw / delta - 1 / delta *
        acc_y + 2 * cos(heading) * sin(heading) * sensor_vely * sensor_velw / delta; // Angular acceleration around z axis

    std_msgs::msg::Float32 u_msg;
    u_msg.data = u;
    u_publisher_->publish(u_msg);

    std_msgs::msg::Float32 a_msg;
    a_msg.data = a;
    a_publisher_->publish(a_msg);

    std_msgs::msg::Float32 alpha_msg;
    alpha_msg.data = alpha;
    alpha_publisher_->publish(alpha_msg);

    cmd_vel.twist.linear.x = 1.0*(prev_sensor_velx + acc_x*dt + px_gain*(a-acc_x)*pow(dt,2.0));
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 1.0*(prev_sensor_velw + acc_w*dt + pw_gain*(alpha-acc_w)*pow(dt,2.0));

  }

  last_odom_ = odom;

  setOutput("command_velocity", cmd_vel);
  return BT::NodeStatus::RUNNING;
}


void BaxevaniController::onHalted()
{
  
}


} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::BaxevaniController>("BaxevaniController");
}
