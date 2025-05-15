#include "project11_navigation/plugins/action/follow_path_cancel_node.h"

namespace project11_navigation
{

FollowPathCancel::FollowPathCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfig & config)
: nav2_behavior_tree::BtCancelActionNode<nav2_msgs::action::FollowPath>(
    xml_tag_name, action_name, config)
{
}

}  // namespace project11_navigation

