#include "marine_nav_behavior_tree/plugins/action/sonar_coverage_action.h"

namespace marine_nav_behavior_tree
{

SonarCoverageAction::SonarCoverageAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfig& config)
: nav2_behavior_tree::BtActionNode<marine_nav_interfaces::action::ComputeSonarCoveragePath>(xml_tag_name, action_name, config)
{
}

void SonarCoverageAction::initialize()
{
  auto survey_area_bb_ = getInput<geometry_msgs::msg::PolygonStamped>("survey_area");
  if(!survey_area_bb_)
  {
    throw BT::RuntimeError(name(), " missing required input [survey_area]: ", survey_area_bb_.error() );
  }
  goal_.survey_area = survey_area_bb_.value();
}

void SonarCoverageAction::on_tick()
{
  if(!BT::isStatusActive(status()))
  {
    initialize();
  }
}

BT::NodeStatus SonarCoverageAction::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SonarCoverageAction::on_aborted()
{
  setOutput("error_message", result_.result->status);  
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SonarCoverageAction::on_cancelled()
{
  return BT::NodeStatus::SUCCESS;
}

void SonarCoverageAction::on_wait_for_result(std::shared_ptr<const typename Action::Feedback> feedback)
{
  if(feedback)
  {
    setOutput("current_line", feedback->current_line);
    setOutput("current_line_number", feedback->line_number);
    std::stringstream label_ss;
    label_ss << "line_" << std::setfill('0') << std::setw(3) << feedback->line_number;
    setOutput("current_line_label", label_ss.str());
  }
}

} // namespace marine_nav_behavior_tree

