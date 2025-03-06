#ifndef PROJECT11_NAVIGATION_ACTIONS_HOVER_ACTION_H
#define PROJECT11_NAVIGATION_ACTIONS_HOVER_ACTION_H

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "project11_nav_msgs/action/hover.hpp"

namespace project11_navigation
{

class HoverAction: public nav2_behavior_tree::BtActionNode<project11_nav_msgs::action::Hover>
{
  using Action = project11_nav_msgs::action::Hover;

public:
  HoverAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfig & config);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  void initialize();

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<double>("minimum_distance", 0.0, "Distance within which robot is deemed at the target and drifts"),
      BT::InputPort<double>("maximum_distance", 0.0, "Distance at which robot uses maximum speed to get to the target"),
      BT::InputPort<double>("maximum_speed", 0.0, "Maximum speed to use to get to the target"),
      BT::OutputPort<Action::Result::_error_code_type>("error_code_id", "The hover server error code")
    });
  }

};


} // namespace project11_navigation

#endif
