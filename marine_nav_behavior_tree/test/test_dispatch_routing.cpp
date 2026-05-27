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

  marine_nav_tasks::TaskPtr makeTask(const std::string & id)
  {
    Task::TaskInformation msg;
    msg.id = id;
    msg.type = "survey_line";
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
