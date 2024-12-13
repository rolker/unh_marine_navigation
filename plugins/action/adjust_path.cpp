#include "project11_navigation/plugins/action/adjust_path.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "project11_navigation/occupancy_grid.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

extern "C" {
#include "dubins_curves/dubins.h"
};


namespace project11_navigation
{

AdjustPath::AdjustPath(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList AdjustPath::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("path", "{path}", "Path needing adjustment"),
    BT::InputPort<int>("current_segment", "{current_segment}", "Current path segment being followed"),
    BT::InputPort<double>("along_track_progress", "{current_segment_along_track_progress}", "How far along the current segment we are"),
    BT::InputPort<nav_msgs::msg::Odometry>("odometry", "{odometry}", "Robot's current odometry state"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer", "{tf_buffer}", "Transform buffer"),
    BT::InputPort<std::shared_ptr<OccupancyGrid> >("local_costmap", "{local_costmap}", "Local costmap used to adjust trajectory"),
    BT::InputPort<double>("turn_radius", "{turn_radius}", "Robot's turn radius"),

    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("adjusted_path", "{adjusted_path}", "Path modified to avoid obstacles"),
    BT::OutputPort<int>("adjusted_current_segment", "{adjusted_current_segment}", "Current segment of the adjusted path"),
    BT::OutputPort<double>("adjusted_cross_track_error", "{adjusted_cross_track_error}", "Cross-track error on adjusted segment"),
    BT::OutputPort<double>("adjusted_along_track_progress", "{adjusted_along_track_progress}", "Progress along adjusted segment"),
    BT::OutputPort<int>("adjusted_segment_count", "{adjusted_segment_count}", "Number of segment in adjusted path")
  };
}

BT::NodeStatus AdjustPath::tick()
{
  auto path_input = getInput<  std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > >("path");
  if(!path_input)
  {
    throw BT::RuntimeError(name(), " missing required input [path]: ", path_input.error() );
  }

  auto odom = getInput<nav_msgs::msg::Odometry>("odometry");
  if(!odom)
  {
    throw BT::RuntimeError("missing required input [odometry]: ", odom.error() );
  }

  auto costmap_input = getInput<std::shared_ptr<OccupancyGrid> >("local_costmap");
  if(!costmap_input)
  {
    throw BT::RuntimeError("missing required input [local_costmap]: ", costmap_input.error() );
  }
  auto costmap = costmap_input.value();

  auto along_track_progress = getInput<double>("along_track_progress");
  if(!along_track_progress)
  {
    throw BT::RuntimeError("missing required input [along_track_progress]: ", costmap_input.error() );
  }

  auto turn_radius = getInput<double>("turn_radius");
  if(!turn_radius)
  {
    throw BT::RuntimeError(name(), " missing required input [turn_radius]: ", turn_radius.error() );
  }


  std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped> > adjusted_path;
  auto path = path_input.value();

  int total_segment_count = 0;

  if(path)
  {
    adjusted_path = std::make_shared<std::vector<geometry_msgs::msg::PoseStamped> >();
    auto current_segment = getInput<int>("current_segment");
    if(current_segment)
    {

      auto segment_index = current_segment.value();
      auto current_progress = along_track_progress.value();

      double step_size = 2.0;

      while (segment_index < path->size()-1)
      {
        auto p1 = path->at(segment_index);
        auto p2 = path->at(segment_index+1);

        auto segment_dx = p2.pose.position.x - p1.pose.position.x;
        auto segment_dy = p2.pose.position.y - p1.pose.position.y;

        auto segment_length = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

        if(current_progress < segment_length)
        {
          p1.pose.position.x += current_progress*segment_dx/segment_length;
          p1.pose.position.y += current_progress*segment_dy/segment_length;

          if(adjusted_path->empty())
            p1.pose = odom.value().pose.pose;
          adjusted_path->push_back(p1);

          current_progress += step_size; 

        }
        else
        {
          segment_index += 1;
          current_progress -= segment_length;
        }

        if(adjusted_path->size() > 10)
          break;
      }

      double total_cost = 0;
      if(!adjusted_path->empty())
      {
        auto last_position = adjusted_path->front().pose.position;
        for(const auto& p: *adjusted_path)
        {
          auto v = int(costmap->getValue(p.pose.position));

          auto dx = p.pose.position.x - last_position.x;
          auto dy = p.pose.position.y - last_position.y;
          auto delta = sqrt(dx*dx+dy*dy);
          total_cost += delta;

          if(v > 0)
            total_cost += v*delta;
        }
        //ROS_DEBUG_STREAM("total cost: " << total_cost);

        if(adjusted_path->size() > 2)
        {
          double start[3];
          start[0] = odom.value().pose.pose.position.x;
          start[1] = odom.value().pose.pose.position.y;
          start[2] = tf2::getYaw(odom.value().pose.pose.orientation);
          
          double target[3];
          target[0] = adjusted_path->back().pose.position.x;
          target[1] = adjusted_path->back().pose.position.y;

          auto dx = target[0] - adjusted_path->at(adjusted_path->size()-2).pose.position.x;
          auto dy = target[1] - adjusted_path->at(adjusted_path->size()-2).pose.position.y;
          target[2] = atan2(dy,dx);

          DubinsPath path;

          if(dubins_shortest_path(&path, start, target, turn_radius.value()) == 0)
          {
            auto path_length = dubins_path_length(&path);
            auto dubins_adjustment = std::make_shared<std::vector<geometry_msgs::msg::PoseStamped> >();
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = odom.value().header.frame_id;
            pose.pose.position.x = start[0];
            pose.pose.position.y = start[1];
            tf2::Quaternion quat;
            quat.setRPY(0,0,start[2]);
            pose.pose.orientation = tf2::toMsg(quat);
            dubins_adjustment->push_back(pose);
            double current_length = 0.0;
            while(current_length < path_length)
            {
              double q[3];
              if(dubins_path_sample(&path, current_length, q) != 0)
                break;
              pose.pose.position.x = q[0];
              pose.pose.position.y = q[1];
              quat.setRPY(0,0,q[2]);
              pose.pose.orientation = tf2::toMsg(quat);
              dubins_adjustment->push_back(pose);
              current_length += step_size;
            }
            pose.pose.position.x = target[0];
            pose.pose.position.y = target[1];
            quat.setRPY(0,0,target[2]);
            pose.pose.orientation = tf2::toMsg(quat);
            dubins_adjustment->push_back(pose);

            adjusted_path = dubins_adjustment;
          }

        }

      }
    }

    total_segment_count = adjusted_path->size()-1;
  }

  setOutput("adjusted_path", adjusted_path);
  setOutput("adjusted_current_segment", 0);
  setOutput("adjusted_cross_track_error", 0.0);
  setOutput("adjusted_along_track_progress", 0.0);
  setOutput("adjusted_segment_count", total_segment_count);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::AdjustPath>("AdjustPath");
}

