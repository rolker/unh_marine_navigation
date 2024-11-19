#include "project11_navigation/plugins/action/update_current_segment.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "project11/utils.h"

namespace project11_navigation
{

UpdateCurrentSegment::UpdateCurrentSegment(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
  cross_track_error_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("cross_track_error", 1);
}

BT::PortsList UpdateCurrentSegment::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("navigation_path"),
    BT::InputPort<nav_msgs::msg::Odometry>("odometry"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::BidirectionalPort<int>("current_segment"),
    BT::OutputPort<double>("segment_length"),
    BT::OutputPort<double>("cross_track_error"),
    BT::OutputPort<double>("along_track_progress"),
    BT::OutputPort<int>("segment_count")
  };
}

BT::NodeStatus UpdateCurrentSegment::tick()
{
  auto navigation_path_bb = getInput<std::shared_ptr<std::vector< geometry_msgs::msg::PoseStamped> > >("navigation_path");
  if(!navigation_path_bb || !navigation_path_bb.value())
  {
    return BT::NodeStatus::FAILURE;
  }
  auto navigation_path = navigation_path_bb.value();

  auto odom = getInput<nav_msgs::msg::Odometry>("odometry");
  if(!odom)
  {
    throw BT::RuntimeError("missing required input [odometry]: ", odom.error() );
  }

  auto tf_buffer = getInput<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer");
  if(!tf_buffer)
  {
    throw BT::RuntimeError("missing required input [tf_buffer]: ", tf_buffer.error() );
  }

  int segment_count = std::max<int>(0,navigation_path->size()-1);
  setOutput("segment_count", segment_count);

  int current_segment = 0;
  auto current_segment_bb = getInput<int>("current_segment");
  if(current_segment_bb)
    current_segment = current_segment_bb.value();

  if(current_segment < 0 || current_segment > segment_count)
    return BT::NodeStatus::FAILURE;

  double segment_length = 0.0;
  double along_track_progress = 0.0;
  double cross_track_error = 0.0;

  if(segment_count > 0)
  {
    geometry_msgs::msg::TransformStamped base_to_map;
    try
    {
      base_to_map = tf_buffer.value()->lookupTransform(navigation_path->front().header.frame_id , odom.value().child_frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "UpdateCurrentSegment node named " << name() << " Error getting path to base_frame transform: " << ex.what());
      return BT::NodeStatus::FAILURE;
    }

    while(current_segment < segment_count)
    {
      auto p1 = (*navigation_path)[current_segment];
      auto p2 = (*navigation_path)[current_segment+1];

      auto segment_dx = p2.pose.position.x - p1.pose.position.x;
      auto segment_dy = p2.pose.position.y - p1.pose.position.y;

      project11::AngleRadians segment_azimuth(atan2(segment_dy, segment_dx));
      segment_length = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

      // vehicle distance and azimuth relative to the segment's start point
      double dx = p1.pose.position.x - base_to_map.transform.translation.x;
      double dy = p1.pose.position.y - base_to_map.transform.translation.y;
      auto vehicle_distance = sqrt(dx*dx+dy*dy);

      project11::AngleRadians vehicle_azimuth(atan2(-dy, -dx));

      auto error_azimuth = vehicle_azimuth - segment_azimuth;
      
      auto sin_error_azimuth = sin(error_azimuth);
      auto cos_error_azimuth = cos(error_azimuth);

      // Distance traveled along the line.
      along_track_progress = vehicle_distance*cos_error_azimuth;

      if(along_track_progress > segment_length)
      {
        // segment done, move to the next one
        current_segment++;
      }
      else
      {
        // segment is not done, so this is the current one
        cross_track_error = vehicle_distance*sin_error_azimuth;
        break;
      }
    }

    // 0 based counting check if we are past the last segment
    if(current_segment == segment_count)
    {
      segment_length = 0.0;
      along_track_progress = 0.0;
      cross_track_error = 0.0;
    }
  }

  setOutput("current_segment", current_segment);
  setOutput("segment_length", segment_length);
  setOutput("cross_track_error", cross_track_error);
  std_msgs::msg::Float32 cte_msg;
  cte_msg.data = cross_track_error;
  cross_track_error_publisher_->publish(cte_msg);
  setOutput("along_track_progress", along_track_progress);
  
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::UpdateCurrentSegment>("UpdateCurrentSegment");
}
