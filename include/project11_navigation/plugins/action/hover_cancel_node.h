#ifndef PROJECT11_NAVIGATION_ACTIONS_HOVER_CANCEL_NODE_H
#define PROJECT11_NAVIGATION_ACTIONS_HOVER_CANCEL_NODE_H

#include "project11_nav_msgs/action/hover.hpp"
#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace project11_navigation
{

class HoverCancel : public nav2_behavior_tree::BtCancelActionNode<project11_nav_msgs::action::Hover>
{
public:
  HoverCancel(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
    });
  }
};

} // namespace project11_navigation

#endif

