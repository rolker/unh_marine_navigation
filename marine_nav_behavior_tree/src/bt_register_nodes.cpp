#include "behaviortree_cpp/bt_factory.h"

#include "marine_nav_behavior_tree/plugins/action/add_sub_task.h"
#include "marine_nav_behavior_tree/plugins/action/clear_path.h"
#include "marine_nav_behavior_tree/plugins/action/fix_path_orientations.h"
#include "marine_nav_behavior_tree/plugins/action/follow_path_cancel_node.h"
#include "marine_nav_behavior_tree/plugins/action/get_sub_path.h"
#include "marine_nav_behavior_tree/plugins/action/get_sub_tasks.h"
#include "marine_nav_behavior_tree/plugins/action/get_task_data_double.h"
#include "marine_nav_behavior_tree/plugins/action/get_task_data_string.h"
#include "marine_nav_behavior_tree/plugins/action/hover_action.h"
#include "marine_nav_behavior_tree/plugins/action/hover_cancel_node.h"
#include "marine_nav_behavior_tree/plugins/action/sonar_coverage_action.h"
#include "marine_nav_behavior_tree/plugins/action/sonar_coverage_cancel_node.h"
#include "marine_nav_behavior_tree/plugins/action/path_to_pose_vector.h"
#include "marine_nav_behavior_tree/plugins/action/set_controller_speed.h"
#include "marine_nav_behavior_tree/plugins/action/set_path_from_task.h"
#include "marine_nav_behavior_tree/plugins/action/set_polygon_from_task.h"
#include "marine_nav_behavior_tree/plugins/action/set_task_done.h"
#include "marine_nav_behavior_tree/plugins/action/update_current_task.h"

#include "marine_nav_behavior_tree/plugins/condition/all_tasks_done.h"
#include "marine_nav_behavior_tree/plugins/condition/has_sub_tasks.h"
#include "marine_nav_behavior_tree/plugins/condition/path_empty.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<marine_nav_behavior_tree::AddSubTask>("AddSubTask");
  factory.registerNodeType<marine_nav_behavior_tree::ClearPath>("ClearPath");
  factory.registerNodeType<marine_nav_behavior_tree::FixPathOrientations>("FixPathOrientations");

  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<marine_nav_behavior_tree::FollowPathCancel>(
        name, "follow_path", config);
    };

  factory.registerBuilder<marine_nav_behavior_tree::FollowPathCancel>(
    "CancelFollowPath", builder);

  factory.registerNodeType<marine_nav_behavior_tree::GetSubPath>("GetSubPath");
  factory.registerNodeType<marine_nav_behavior_tree::GetSubTasks>("GetSubTasks");
  factory.registerNodeType<marine_nav_behavior_tree::GetTaskDataDouble>("GetTaskDataDouble");
  factory.registerNodeType<marine_nav_behavior_tree::GetTaskDataString>("GetTaskDataString");

  BT::NodeBuilder hover_builder = 
  [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<marine_nav_behavior_tree::HoverAction>(name, "hover", config);
  };
  factory.registerBuilder<marine_nav_behavior_tree::HoverAction>("Hover", hover_builder);

  BT::NodeBuilder hover_cancel_builder =
  [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<marine_nav_behavior_tree::HoverCancel>(name, "hover", config);
  };

  factory.registerBuilder<marine_nav_behavior_tree::HoverCancel>("CancelHover", hover_cancel_builder);


  BT::NodeBuilder multibeam_coverage_builder = 
  [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<marine_nav_behavior_tree::SonarCoverageAction>(name, "compute_sonar_coverage_path", config);
  };
  factory.registerBuilder<marine_nav_behavior_tree::SonarCoverageAction>("SonarCoverage", multibeam_coverage_builder);

  BT::NodeBuilder multibeam_coverage_cancel_builder =
  [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<marine_nav_behavior_tree::SonarCoverageCancel>(name, "compute_sonar_coverage_path", config);
  };
  factory.registerBuilder<marine_nav_behavior_tree::SonarCoverageCancel>("CancelSonarCoverage", multibeam_coverage_cancel_builder);

  factory.registerNodeType<marine_nav_behavior_tree::PathToPoseVector>("PathToPoseVector");
  factory.registerNodeType<marine_nav_behavior_tree::SetControllerSpeed>("SetControllerSpeed");
  factory.registerNodeType<marine_nav_behavior_tree::SetPathFromTask>("SetPathFromTask");
  factory.registerNodeType<marine_nav_behavior_tree::SetPolygonFromTask>("SetPolygonFromTask");
  factory.registerNodeType<marine_nav_behavior_tree::SetTaskDone>("SetTaskDone");
  factory.registerNodeType<marine_nav_behavior_tree::UpdateCurrentTask>("UpdateCurrentTask");

  factory.registerNodeType<marine_nav_behavior_tree::AllTasksDone>("AllTasksDoneCondition");
  factory.registerNodeType<marine_nav_behavior_tree::HasSubTasks>("HasSubTasksCondition");
  factory.registerNodeType<marine_nav_behavior_tree::PathEmpty>("PathEmptyCondition");

}
