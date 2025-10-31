#include "marine_nav_behavior_tree/plugins/action/sonar_coverage_cancel_node.h"

namespace marine_nav_behavior_tree
{

SonarCoverageCancel::SonarCoverageCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfig & config)
: BtCancelActionNode(xml_tag_name, action_name, config)
{
}

}  // namespace marine_nav_behavior_tree
