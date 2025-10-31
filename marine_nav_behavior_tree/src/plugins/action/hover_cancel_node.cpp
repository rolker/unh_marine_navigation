#include "marine_nav_behavior_tree/plugins/action/hover_cancel_node.h"

namespace marine_nav_behavior_tree
{

HoverCancel::HoverCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfig & config)
: BtCancelActionNode<marine_nav_interfaces::action::Hover>(xml_tag_name, action_name, config)
{
}

} // namespace marine_nav_behavior_tree

