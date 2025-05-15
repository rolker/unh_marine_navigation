#include "project11_navigation/plugins/action/hover_action.h"

namespace project11_navigation
{

HoverAction::HoverAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfig& config)
: BtActionNode<project11_nav_msgs::action::Hover>(xml_tag_name, action_name, config)
{
}

void HoverAction::initialize()
{
  double minimum_radius;
  getInput("minimum_radius", minimum_radius);
  double maximum_radius;
  getInput("maximum_radius", maximum_radius);
  double maximum_speed;
  getInput("maximum_speed", maximum_speed);

  goal_.minimum_radius = minimum_radius;
  goal_.maximum_radius = maximum_radius;
  goal_.maximum_speed = maximum_speed;

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



} // project11_navigation

