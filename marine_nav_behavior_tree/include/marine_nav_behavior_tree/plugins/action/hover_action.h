#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_HOVER_ACTION_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_HOVER_ACTION_H

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "marine_nav_interfaces/action/hover.hpp"

namespace marine_nav_behavior_tree
{

class HoverAction: public nav2_behavior_tree::BtActionNode<marine_nav_interfaces::action::Hover>
{
  using Action = marine_nav_interfaces::action::Hover;

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
      BT::InputPort<double>("minimum_radius", 0.0, "Radius within which robot is deemed at the target and drifts"),
      BT::InputPort<double>("maximum_radius", 0.0, "Radius at which robot uses maximum speed to get to the target"),
      BT::InputPort<double>("maximum_speed", 0.0, "Maximum speed to use to get to the target"),
      BT::OutputPort<Action::Result::_error_code_type>("error_code_id", "The hover server error code")
    });
  }

};


} // namespace marine_nav_behavior_tree

#endif
