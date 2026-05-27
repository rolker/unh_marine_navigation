#include "marine_nav_behavior_tree/plugins/action/hover_action.h"

namespace marine_nav_behavior_tree
{

HoverAction::HoverAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfig& config)
: BtActionNode<marine_nav_interfaces::action::Hover>(xml_tag_name, action_name, config)
{
}

void HoverAction::initialize()
{
  double minimum_radius = 0.0;
  getInput("minimum_radius", minimum_radius);
  double maximum_radius = 0.0;
  getInput("maximum_radius", maximum_radius);
  double maximum_speed = 0.0;
  getInput("maximum_speed", maximum_speed);

  goal_.minimum_radius = minimum_radius;
  goal_.maximum_radius = maximum_radius;
  goal_.maximum_speed = maximum_speed;

  // Optional hold pose. When the port is unset (e.g. the post-transit
  // location branch never writes {hover_target}), leave target with an empty
  // frame_id — the Hover behavior treats that as "hold the current pose".
  goal_.target = geometry_msgs::msg::PoseStamped();
  getInput("target", goal_.target);

}

void HoverAction::on_tick()
{
  if(!BT::isStatusActive(status()))
  {
    initialize();
  }
}

BT::NodeStatus HoverAction::on_success()
{
  setOutput("error_code_id", Action::Result::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus HoverAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus HoverAction::on_cancelled()
{
  setOutput("error_code_id", Action::Result::NONE);
  return BT::NodeStatus::SUCCESS;
}



} // namespace marine_nav_behavior_tree

