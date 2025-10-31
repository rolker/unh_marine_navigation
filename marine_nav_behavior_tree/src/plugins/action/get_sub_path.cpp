#include "marine_nav_behavior_tree/plugins/action/get_sub_path.h"

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
  auto input_path = input_path_bb.value();

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

  auto start_index = start_index_bb.value();
  auto end_index = end_index_bb.value();

  if(end_index < 0)
  {
    end_index = input_path.poses.size() + end_index;
  }

  nav_msgs::msg::Path output_path;

  for(std::size_t i = start_index; i <= end_index && i < input_path.poses.size(); i++)
  {
    output_path.poses.push_back(input_path.poses[i]);
  }

  if(!output_path.poses.empty())
  {
    output_path.header = output_path.poses.front().header;
  }

  setOutput("output_path", output_path);

  return BT::NodeStatus::SUCCESS;
}

} // namespace marine_nav_behavior_tree
