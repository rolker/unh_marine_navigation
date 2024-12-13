#include "project11_navigation/utilities.h"
#include "project11/utils.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace project11_navigation
{

void adjustTrajectoryForSpeed(std::vector<geometry_msgs::msg::PoseStamped>& trajectory, double speed)
{
  double total_distance = 0.0;
  auto last = trajectory.begin();
  auto next = last;
  if(next != trajectory.end())
    next++;
  while(next != trajectory.end())
  {
    double dx = next->pose.position.x - last->pose.position.x;
    double dy = next->pose.position.y - last->pose.position.y;
    double distance = sqrt(dx*dx+dy*dy);
    total_distance += distance;
    next->header.stamp = trajectory.front().header.stamp+rclcpp::Duration::from_seconds(total_distance/speed);
    last = next;
    next++;
  }

}

void adjustPathOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path)
{
  auto last = path.begin();
  auto next = last;
  if(next != path.end())
    next++;
  while(next != path.end())
  {
    gz4d::Vector<double, 2> last_point(last->pose.position.x, last->pose.position.y);
    gz4d::Vector<double, 2> next_point(next->pose.position.x, next->pose.position.y);
    auto diff = next_point-last_point;
    auto unit_diff = normalize(diff);
    auto cos_theta = unit_diff.dot(gz4d::Vector<double, 2>(1.0, 0.0));
    auto theta = acos(cos_theta);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    last->pose.orientation = tf2::toMsg(q);
    next->pose.orientation = last->pose.orientation;
    last = next;
    next++;
  }
}

geometry_msgs::msg::Vector3 vectorBetween(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to)
{
  geometry_msgs::msg::Vector3 ret;

  ret.x = to.position.x - from.position.x;
  ret.y = to.position.y - from.position.y;
  ret.z = to.position.z - from.position.z;

  return ret;
}

double length(const geometry_msgs::msg::Vector3& vector)
{
  double sum = vector.x*vector.x + vector.y*vector.y + vector.z*vector.z;
  if(sum > 0.0)
    return sqrt(sum);
  return 0.0;
}

geometry_msgs::msg::Vector3 normalize(const geometry_msgs::msg::Vector3& vector)
{
  geometry_msgs::msg::Vector3 ret;
  double l = length(vector);
  if(l>0.0)
  {
    ret.x = vector.x/l;
    ret.y = vector.y/l;
    ret.z = vector.z/l;
  }
  return ret;
}

// double readDoubleOrIntParameter(ros::NodeHandle &nh, const std::string& parameter, double default_value)
// {
//   XmlRpc::XmlRpcValue value;
//   if(nh.getParam(parameter, value))
//   {
//     if(value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
//       return static_cast<double>(value);
//     if(value.getType() == XmlRpc::XmlRpcValue::TypeInt)
//       return static_cast<int>(value);
//     ROS_FATAL_STREAM("Expected number for parameter " << parameter << " but got " << std::string(value));
//     throw std::runtime_error("Could not read number from parameter");
//   }
//   return default_value;
// }


} // namespace project11_navigation
