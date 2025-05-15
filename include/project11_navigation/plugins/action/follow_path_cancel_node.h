#ifndef PROJECT11_NAVIGATION_ACTIONS_FOLLOW_PATH_CANCEL_NODE_H
#define PROJECT11_NAVIGATION_ACTIONS_FOLLOW_PATH_CANCEL_NODE_H


#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace project11_navigation
{

class FollowPathCancel : public nav2_behavior_tree::BtCancelActionNode<nav2_msgs::action::FollowPath>
{
public:
  FollowPathCancel(
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

#endif // PROJECT11_NAVIGATION_ACTIONS_FOLLOW_PATH_CANCEL_NODE_H