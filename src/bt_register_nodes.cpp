#include "behaviortree_cpp/bt_factory.h"

#include "project11_navigation/plugins/action/baxevani_controller.h"
#include "project11_navigation/plugins/action/clear_path.h"
#include "project11_navigation/plugins/action/crabbing_path_follower.h"
#include "project11_navigation/plugins/action/debug_blackboard.h"
#include "project11_navigation/plugins/action/follow_path_cancel_node.h"
#include "project11_navigation/plugins/action/generate_plan.h"
#include "project11_navigation/plugins/action/get_sub_tasks.h"
#include "project11_navigation/plugins/action/get_task_data_double.h"
#include "project11_navigation/plugins/action/get_task_data_string.h"
#include "project11_navigation/plugins/action/hover_action.h"
#include "project11_navigation/plugins/action/hover_cancel_node.h"
#include "project11_navigation/plugins/action/multibeam_coverage_action.h"
#include "project11_navigation/plugins/action/pose_vector_to_path.h"
#include "project11_navigation/plugins/action/predict_stopping_pose.h"
#include "project11_navigation/plugins/action/set_pose_from_task.h"
#include "project11_navigation/plugins/action/set_task_done.h"
#include "project11_navigation/plugins/action/set_trajectory_from_task.h"
#include "project11_navigation/plugins/action/task_list_updater.h"
#include "project11_navigation/plugins/action/update_current_segment.h"
#include "project11_navigation/plugins/action/update_current_task.h"
#include "project11_navigation/plugins/action/visualize_trajectory.h"

#include "project11_navigation/plugins/condition/all_tasks_done.h"
#include "project11_navigation/plugins/condition/goal_reached.h"
#include "project11_navigation/plugins/condition/path_empty.h"
#include "project11_navigation/plugins/condition/plan_needed.h"
#include "project11_navigation/plugins/condition/task_updated.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::BaxevaniController>("BaxevaniController");
  factory.registerNodeType<project11_navigation::ClearPath>("ClearPath");
  factory.registerNodeType<project11_navigation::CrabbingPathFollower>("CrabbingPathFollower");
  factory.registerNodeType<project11_navigation::DebugBlackboardString>("DebugBlackboardString");
  factory.registerNodeType<project11_navigation::DebugBlackboardDouble>("DebugBlackboardDouble");

  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<project11_navigation::FollowPathCancel>(
        name, "follow_path", config);
    };

  factory.registerBuilder<project11_navigation::FollowPathCancel>(
    "CancelFollowPath", builder);

  factory.registerNodeType<project11_navigation::GeneratePlan>("GeneratePlan");
  factory.registerNodeType<project11_navigation::GetSubTasks>("GetSubTasks");
  factory.registerNodeType<project11_navigation::GetTaskDataDouble>("GetTaskDataDouble");
  factory.registerNodeType<project11_navigation::GetTaskDataString>("GetTaskDataString");

  BT::NodeBuilder hover_builder = 
  [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<project11_navigation::HoverAction>(name, "hover", config);
  };
  factory.registerBuilder<project11_navigation::HoverAction>("Hover", hover_builder);

  BT::NodeBuilder hover_cancel_builder =
  [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<project11_navigation::HoverCancel>(name, "hover", config);
  };

  factory.registerBuilder<project11_navigation::HoverCancel>("CancelHover", hover_cancel_builder);

  factory.registerNodeType<project11_navigation::MultibeamCoverageActionCancel>("MultibeamCoverageActionCancel");
  factory.registerNodeType<project11_navigation::MultibeamCoverageActionDoneCondition>("MultibeamCoverageActionDoneCondition");
  factory.registerNodeType<project11_navigation::MultibeamCoverageActionSetGoal>("MultibeamCoverageActionSetGoal");
  factory.registerNodeType<project11_navigation::MultibeamCoverageActionUpdateTask>("MultibeamCoverageActionUpdateTask");
  factory.registerNodeType<project11_navigation::PoseVectorToPath>("PoseVectorToPath");
  factory.registerNodeType<project11_navigation::PredictStoppingPose>("PredictStoppingPose");
  factory.registerNodeType<project11_navigation::SetPoseFromTask>("SetPoseFromTask");
  factory.registerNodeType<project11_navigation::SetTaskDone>("SetTaskDone");
  factory.registerNodeType<project11_navigation::SetTrajectoryFromTask>("SetTrajectoryFromTask");
  factory.registerNodeType<project11_navigation::TaskListUpdater>("TaskListUpdater");
  factory.registerNodeType<project11_navigation::UpdateCurrentSegment>("UpdateCurrentSegment");
  factory.registerNodeType<project11_navigation::UpdateCurrentTask>("UpdateCurrentTask");
  factory.registerNodeType<project11_navigation::VisualizeTrajectory>("VisualizeTrajectory");

  factory.registerNodeType<project11_navigation::AllTasksDone>("AllTasksDoneCondition");
  factory.registerNodeType<project11_navigation::GoalReached>("GoalReachedCondition");
  factory.registerNodeType<project11_navigation::PathEmpty>("PathEmptyCondition");
  factory.registerNodeType<project11_navigation::PlanNeeded>("PlanNeededCondition");
  factory.registerNodeType<project11_navigation::TaskUpdated>("TaskUpdatedCondition");

}
