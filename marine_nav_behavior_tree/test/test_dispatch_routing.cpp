// Regression guard for #25's core dispatch distinction: with a Switch-on-type +
// default-catchall + per-case Fallback[<task>, SetTaskFailed] shape (as used in
// run_tasks.xml's NavigatorSequence and the survey-area loop), a MATCHED-but-FAILED
// task is recorded failed (status set) instead of falling through to the catchall and
// being silently marked done, while an UNMATCHED type still reaches the clean-done
// catchall. This tests the pattern with stub leaves; full-tree integration (mid-run
// reactivity, the real nav2 nodes) is deferred to #8's harness.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "marine_nav_behavior_tree/plugins/action/set_task_failed.h"
#include "marine_nav_behavior_tree/plugins/action/set_task_done.h"
#include "marine_nav_tasks/task.h"

using marine_nav_behavior_tree::SetTaskDone;
using marine_nav_behavior_tree::SetTaskFailed;
using marine_nav_tasks::Task;

class DispatchRoutingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("dispatch_routing_test");
    factory_.registerNodeType<SetTaskFailed>("SetTaskFailed");
    factory_.registerNodeType<SetTaskDone>("SetTaskDone");
    // Stub leaves whose outcome we control, standing in for a task subtree's execution.
    factory_.registerSimpleAction(
      "ExecFails", [](BT::TreeNode &) {return BT::NodeStatus::FAILURE;});
    factory_.registerSimpleAction(
      "ExecSucceeds", [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;});
  }

  marine_nav_tasks::TaskPtr makeTask(
    const std::string & id, const std::string & type = "survey_line")
  {
    Task::TaskInformation msg;
    msg.id = id;
    msg.type = type;
    return Task::create(msg, node_->get_clock());
  }

  // Mirror of run_tasks.xml's dispatch shape: case_1 (survey_line) executes a leaf that
  // FAILS -> Fallback to SetTaskFailed; case_2 (goto) executes a leaf that SUCCEEDS ->
  // SetTaskDone; default -> clean SetTaskDone catchall.
  BT::NodeStatus runDispatch(const std::string & type, const marine_nav_tasks::TaskPtr & task)
  {
    auto bb = BT::Blackboard::create();
    bb->set("node", node_);
    bb->set("task", task);
    bb->set("type", type);
    auto tree = factory_.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree>)"
      R"(<Switch2 variable="{type}" case_1="survey_line" case_2="goto">)"
      R"(  <Fallback><Sequence><ExecFails/><SetTaskDone task="{task}"/></Sequence>)"
      R"(    <SetTaskFailed task="{task}" reason="exec_failed"/></Fallback>)"
      R"(  <Fallback><Sequence><ExecSucceeds/><SetTaskDone task="{task}"/></Sequence>)"
      R"(    <SetTaskFailed task="{task}"/></Fallback>)"
      R"(  <SetTaskDone name="SkipUnknownTaskType" task="{task}"/>)"
      R"(</Switch2></BehaviorTree></root>)",
      bb);
    return tree.tickWhileRunning();
  }

  // Mirror of the survey-area loop's nested dispatch + the autoremap subtlety it depends on.
  // Inside RunSurveyAreaSubTasks the *sub-task* type lives under current_survey_area_task_type,
  // while current_task_type carries the AREA type ("survey_area", inherited from SurveyAreaTask
  // via _autoremap). Both case_1 (SurveyLineTask, gates on current_task_type == 'survey_line')
  // and case_2 (SurveyLineSetTask, gates on current_task_type == 'survey_line_set') therefore
  // MUST explicitly remap current_task_type="{current_survey_area_task_type}" on the SubTree
  // call, or the inner guard sees the inherited "survey_area", fails, and the Fallback falsely
  // records a correctly-typed sub-task as failed. `with_remap` toggles that remap;
  // `sub_task_type` selects which case (matches the InnerSubTask guard's expected string).
  BT::NodeStatus runNestedAreaSubTask(
    bool with_remap, const std::string & sub_task_type,
    const marine_nav_tasks::TaskPtr & task)
  {
    auto bb = BT::Blackboard::create();
    bb->set("node", node_);
    bb->set("task", task);
    bb->set<std::string>("current_task_type", "survey_area");           // inherited area type
    bb->set<std::string>("current_survey_area_task_type", sub_task_type);  // the sub-task type

    const std::string remap =
      with_remap ? R"( current_task_type="{current_survey_area_task_type}")" : "";
    const std::string xml =
      R"(<root BTCPP_format="4">)"
      R"(<BehaviorTree ID="AreaSubTaskDispatch"><Fallback>)"
      R"(  <Sequence><SubTree ID="InnerSubTask")" + remap + R"( _autoremap="true"/>)"
      R"(    <SetTaskDone task="{task}"/></Sequence>)"
      R"(  <SetTaskFailed task="{task}" reason="sub_task_failed"/>)"
      R"(</Fallback></BehaviorTree>)"
      R"(<BehaviorTree ID="InnerSubTask"><ReactiveSequence>)"
      R"(  <ScriptCondition code="current_task_type == ')" + sub_task_type + R"('"/>)"
      R"(  <ExecSucceeds/>)"
      R"(</ReactiveSequence></BehaviorTree></root>)";
    factory_.registerBehaviorTreeFromText(xml);
    auto tree = factory_.createTree("AreaSubTaskDispatch", bb);
    return tree.tickWhileRunning();
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(DispatchRoutingTest, MatchedButFailedIsRecordedFailedNotSilentlyDone)
{
  // The #25 bug: a matched (survey_line) task whose execution FAILS used to fall through
  // to the catchall and be marked clean-done. Now it must route to SetTaskFailed.
  auto task = makeTask("line_fail");
  EXPECT_EQ(runDispatch("survey_line", task), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  ASSERT_FALSE(task->message().status.empty());
  EXPECT_EQ(YAML::Load(task->message().status)["outcome"].as<std::string>(), "failed");
}

TEST_F(DispatchRoutingTest, UnmatchedTypeReachesCleanDoneCatchall)
{
  // An unknown type is correctly marked done by the default catchall, with empty status.
  auto task = makeTask("unknown");
  EXPECT_EQ(runDispatch("dredge", task), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  EXPECT_TRUE(task->message().status.empty());
}

TEST_F(DispatchRoutingTest, MatchedAndSucceededIsCleanDone)
{
  // A matched task that succeeds is a clean completion: done, empty status (not failed).
  auto task = makeTask("goto_ok");
  EXPECT_EQ(runDispatch("goto", task), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  EXPECT_TRUE(task->message().status.empty());
}

// Regression for the area-loop autoremap bug (Copilot R2 on PR #37) — covers BOTH the
// case_2 (SurveyLineSetTask) call this PR fixed AND case_1 (SurveyLineTask) which carries
// the same load-bearing remap. If either call's explicit current_task_type remap is ever
// dropped, the corresponding "RemapReachesBody" test flips to a "failed" status and fails.

TEST_F(DispatchRoutingTest, NestedSurveyLineSetRemapReachesBodyNotFalseFail)
{
  auto task = makeTask("line_set_ok", "survey_line_set");
  EXPECT_EQ(runNestedAreaSubTask(/*with_remap=*/true, "survey_line_set", task),
            BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  EXPECT_TRUE(task->message().status.empty());  // clean done, NOT failed
}

TEST_F(DispatchRoutingTest, NestedSurveyLineSetWithoutRemapFalselyFails)
{
  // Pins the failure mode the remap fixes: relying on _autoremap alone, the inner guard
  // sees the inherited "survey_area" and fails, routing a correctly-typed line set to
  // SetTaskFailed. Documents the bug class so the remap's necessity is explicit.
  auto task = makeTask("line_set_misremap", "survey_line_set");
  EXPECT_EQ(runNestedAreaSubTask(/*with_remap=*/false, "survey_line_set", task),
            BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  ASSERT_FALSE(task->message().status.empty());
  EXPECT_EQ(YAML::Load(task->message().status)["outcome"].as<std::string>(), "failed");
}

TEST_F(DispatchRoutingTest, NestedSurveyLineRemapReachesBodyNotFalseFail)
{
  // case_1 sibling of the SurveyLineSet test: SurveyLineTask's call in RunSurveyAreaSubTasks
  // (run_tasks.xml:254) ALSO needs the explicit current_task_type remap. Guards that line.
  auto task = makeTask("line_ok", "survey_line");
  EXPECT_EQ(runNestedAreaSubTask(/*with_remap=*/true, "survey_line", task),
            BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  EXPECT_TRUE(task->message().status.empty());
}

TEST_F(DispatchRoutingTest, NestedSurveyLineWithoutRemapFalselyFails)
{
  // Same failure-mode pin for case_1.
  auto task = makeTask("line_misremap", "survey_line");
  EXPECT_EQ(runNestedAreaSubTask(/*with_remap=*/false, "survey_line", task),
            BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  ASSERT_FALSE(task->message().status.empty());
  EXPECT_EQ(YAML::Load(task->message().status)["outcome"].as<std::string>(), "failed");
}
