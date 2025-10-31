#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SONAR_COVERAGE_ACTION_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SONAR_COVERAGE_ACTION_H

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "marine_nav_interfaces/action/compute_sonar_coverage_path.hpp"

namespace marine_nav_behavior_tree
{


class SonarCoverageAction: public nav2_behavior_tree::BtActionNode<marine_nav_interfaces::action::ComputeSonarCoveragePath>
{
  using Action = marine_nav_interfaces::action::ComputeSonarCoveragePath;

public:
  SonarCoverageAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfig & config);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  void on_wait_for_result(std::shared_ptr<const typename Action::Feedback> feedback) override;

  void initialize();

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<geometry_msgs::msg::PolygonStamped>("survey_area", "{survey_area}", "Polygon representing the area to be surveyed"),
      BT::OutputPort<nav_msgs::msg::Path>("current_line", "{current_survey_line}", "Current survey line"),
      BT::OutputPort<int16_t>("current_line_number", "{current_line_number}","Current survey line number"),
      BT::OutputPort<std::string>("current_line_label", "{current_line_label}", "Current survey line label")
    });
  }

};

} // namespace marine_nav_behavior_tree


#endif
