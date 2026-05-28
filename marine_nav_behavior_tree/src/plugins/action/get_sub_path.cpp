#include "marine_nav_behavior_tree/plugins/action/get_sub_path.h"

#include "builtin_interfaces/msg/time.hpp"

namespace marine_nav_behavior_tree
{

GetSubPath::GetSubPath(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::NodeStatus GetSubPath::tick()
{
  auto input_path_bb = getInput<nav_msgs::msg::Path>("input_path");
  if(!input_path_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [input_path]: ", input_path_bb.error() );
  }
  auto start_index_bb = getInput<int>("start_index");
  if(!start_index_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [start_index]: ", start_index_bb.error() );
  }
  auto end_index_bb = getInput<int>("end_index");
  if(!end_index_bb)
  {
    throw BT::RuntimeError(name(), " missing required input [end_index]: ", end_index_bb.error() );
  }

  setOutput("output_path",
            buildSubPath(input_path_bb.value(), start_index_bb.value(), end_index_bb.value()));

  return BT::NodeStatus::SUCCESS;
}

nav_msgs::msg::Path GetSubPath::buildSubPath(
  const nav_msgs::msg::Path& input_path,
  int start_index,
  int end_index)
{
  if(end_index < 0)
  {
    end_index = static_cast<int>(input_path.poses.size()) + end_index;
  }

  nav_msgs::msg::Path output_path;

  // Out-of-range indices yield an empty path. Without this guard, casting a
  // still-negative end_index (e.g. end_index = -poses.size()-1) to size_t in
  // the loop below wraps to a huge value and the loop iterates the whole
  // vector instead — silently honouring an invalid range. Same shape for a
  // negative start_index.
  if(start_index < 0 || end_index < 0 || start_index > end_index)
  {
    return output_path;
  }

  for(std::size_t i = start_index; i <= static_cast<std::size_t>(end_index) && i < input_path.poses.size(); i++)
  {
    output_path.poses.push_back(input_path.poses[i]);
  }

  if(!output_path.poses.empty())
  {
    output_path.header.frame_id = output_path.poses.front().header.frame_id;
    // Zero-stamp idiom (same as path_to_pose_vector.cpp:33, applied here to
    // the outer Path header) tells TF lookups to use "latest". Per-pose
    // stamps stay untouched (load-bearing downstream — see header). For #23.
    output_path.header.stamp = builtin_interfaces::msg::Time();
  }

  return output_path;
}

} // namespace marine_nav_behavior_tree
