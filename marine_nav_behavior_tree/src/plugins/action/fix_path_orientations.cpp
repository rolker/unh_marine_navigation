#include "marine_nav_behavior_tree/plugins/action/fix_path_orientations.h"

#include "marine_nav_utilities/utilities.h"
#include "nav2_util/robot_utils.hpp"

namespace marine_nav_behavior_tree
{

FixPathOrientations::FixPathOrientations(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::NodeStatus FixPathOrientations::tick()
{
  auto input_path_bb = getInput<nav_msgs::msg::Path>("input_path");
  if(!input_path_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [input_path]: ", input_path_bb.error() );
  }
  auto path = input_path_bb.value();


  if(path.poses.size() == 1)
  {
    if(marine_nav_utilities::quaternionSeemsValid(path.poses.front().pose.orientation))
    {
      // Orientation is valid, nothing to do
    }
    else
    {
      auto tf_buffer_bb = getInput<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
      if(!tf_buffer_bb)
      {
        throw BT::RuntimeError(name(), " missing required input [tf_buffer]: ", tf_buffer_bb.error() );
      }
      auto tf_buffer = tf_buffer_bb.value();
      auto robot_frame_bb = getInput<std::string>("robot_frame");
      if(!robot_frame_bb)
      {
        throw BT::RuntimeError(name(), " missing required input [robot_frame]: ", robot_frame_bb.error() );
      }
      auto robot_frame = robot_frame_bb.value();

      geometry_msgs::msg::PoseStamped current_pose;
      if(nav2_util::getCurrentPose(current_pose, *tf_buffer, path.header.frame_id, robot_frame))
      {
        tf2::Vector3 current_position;
        tf2::fromMsg(current_pose.pose.position, current_position);
        tf2::Vector3 path_position;
        tf2::fromMsg(path.poses.front().pose.position, path_position);
        tf2::Vector3 diff = path_position - current_position;
        double yaw = atan2(diff.y(), diff.x());
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        path.poses.front().pose.orientation = tf2::toMsg(q);
      }
    }

  }

  marine_nav_utilities::adjustPathOrientations(path.poses, true);

  setOutput("output_path", path); 

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree

