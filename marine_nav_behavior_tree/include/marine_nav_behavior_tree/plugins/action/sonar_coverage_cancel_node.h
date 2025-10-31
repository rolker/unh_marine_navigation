#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SONAR_COVERAGE_CANCEL_NODE_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_SONAR_COVERAGE_CANCEL_NODE_H

#include "marine_nav_interfaces/action/compute_sonar_coverage_path.hpp"
#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace marine_nav_behavior_tree
{

class SonarCoverageCancel : public nav2_behavior_tree::BtCancelActionNode<marine_nav_interfaces::action::ComputeSonarCoveragePath>
{
public:
  SonarCoverageCancel(
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

} // namespace marine_nav_behavior_tree

#endif

