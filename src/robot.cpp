#include <project11_navigation/robot.h>
#include <project11_navigation/robot_capabilities.h>

namespace project11_navigation
{

Robot::Robot(rclcpp_lifecycle::LifecycleNode::WeakPtr node_ptr):
  Platform(node_ptr)
{
  auto node = node_ptr.lock();
  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("~/cmd_vel", 1);

  enable_sub_ = node->create_subscription<std_msgs::msg::Bool>("~/enable", 10, [this](const std_msgs::msg::Bool::UniquePtr& msg){this->enableCallback(msg);});
}

void Robot::sendControls(const geometry_msgs::msg::TwistStamped& cmd_vel) const
{
  if(enabled_)
    cmd_vel_pub_->publish(cmd_vel);
}

bool Robot::enabled() const
{
  return enabled_;
}

void Robot::enableCallback(const std_msgs::msg::Bool::UniquePtr& msg)
{
  enabled_ = msg->data;
}

void Robot::updateMarkers(visualization_msgs::msg::MarkerArray& marker_array, const geometry_msgs::msg::Polygon& footprint) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = odom_.header.frame_id;
  marker.header.stamp = odom_.header.stamp;
  marker.id = 0;
  marker.ns = odom_.header.frame_id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.pose = odom_.pose.pose;
  marker.color.r = .75;
  marker.color.g = .75;
  marker.color.b = .25;
  marker.color.a = 1.0;
  marker.scale.x = 0.2;
  for(auto p: footprint.points)
  {
    geometry_msgs::msg::Point marker_point;
    marker_point.x = p.x;
    marker_point.y = p.y;
    marker_point.z = p.z;
    marker.points.push_back(marker_point);
  }
  marker_array.markers.push_back(marker);

}

} // namespace project11_navigation
