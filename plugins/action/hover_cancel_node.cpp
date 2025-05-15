#include "project11_navigation/plugins/action/hover_cancel_node.h"

namespace project11_navigation
{

HoverCancel::HoverCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfig & config)
: BtCancelActionNode<project11_nav_msgs::action::Hover>(xml_tag_name, action_name, config)
{
}

} // namespace project11_navigation

