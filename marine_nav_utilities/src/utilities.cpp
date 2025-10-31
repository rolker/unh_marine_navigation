#include "marine_nav_utilities/utilities.h"
#include "marine_nav_utilities/gz4d/vector.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace marine_nav_utilities
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

bool quaternionSeemsValid(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tfq;
  tf2::fromMsg(q, tfq);
  return quaternionSeemsValid(tfq);
}

bool quaternionSeemsValid(const tf2::Quaternion& q)
{
  double len_sq = q.length2();
  return (len_sq > 0.9 && len_sq < 1.1);
}

void adjustPathOrientations(std::vector<geometry_msgs::msg::PoseStamped>& path, bool only_if_invalid)
{
  auto last = path.begin();
  auto next = last;
  if(next != path.end())
    next++;
  while(next != path.end())
  {
    if(only_if_invalid && quaternionSeemsValid(last->pose.orientation))
    {
      last = next;
      next++;
      continue;
    }
    tf2::Vector3 from, to;
    fromMsg(last->pose.position, from);
    fromMsg(next->pose.position, to);

    auto direction_vector = to - from;
    if(direction_vector.length2() > 0.0)
    {
      tf2::Vector3 x_axis(1.0, 0.0, 0.0);

      double cos_theta = direction_vector.normalized().dot(x_axis);
      double theta = acos(cos_theta);

      auto rotation_axis = x_axis.cross(direction_vector);

      tf2::Quaternion q(rotation_axis, theta);
      last->pose.orientation = tf2::toMsg(q);
      if(!only_if_invalid || !quaternionSeemsValid(next->pose.orientation))
      {
        next->pose.orientation = last->pose.orientation;
      }
    }
    last = next;
    next++;
  }
}

template<>
tf2::Vector3 vectorBetween(const geometry_msgs::msg::Pose& from_pose, const geometry_msgs::msg::Pose& to_pose)
{
  return vectorBetween(from_pose.position, to_pose.position);
}


} // namespace marine_nav_utilities
