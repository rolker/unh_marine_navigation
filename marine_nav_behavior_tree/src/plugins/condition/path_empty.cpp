#include "marine_nav_behavior_tree/plugins/condition/path_empty.h"


namespace marine_nav_behavior_tree
{
PathEmpty::PathEmpty(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{
}

BT::NodeStatus PathEmpty::tick()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  if(!path)
  {
    throw BT::RuntimeError("PathEmpty node named ", name(), " missing required input [path]: ", path.error() );
  }

  if(path.value().poses.empty())
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

} // namespace marine_nav_behavior_tree
