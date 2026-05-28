// Unit tests for the SetTaskFailed BT node — it must record a task as
// attempted-but-failed (non-empty structured status) AND mark it done so the
// mission advances, in contrast to SetTaskDone which leaves status empty (a
// clean completion). The status field is what distinguishes a skipped/failed
// line from a completed one in the post-mission coverage record / camp heartbeat.

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

class SetTaskFailedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("set_task_failed_test");
  }

  // Build a fresh survey_line task that is not yet done.
  marine_nav_tasks::TaskPtr makeTask(const std::string & id)
  {
    Task::TaskInformation msg;
    msg.id = id;
    msg.type = "survey_line";
    return Task::create(msg, node_->get_clock());
  }

  BT::Blackboard::Ptr makeBlackboard(const marine_nav_tasks::TaskPtr & task)
  {
    auto bb = BT::Blackboard::create();
    bb->set("node", node_);
    bb->set("task", task);
    return bb;
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(SetTaskFailedTest, RecordsFailedStatusAndMarksDone)
{
  auto task = makeTask("line_1");
  ASSERT_FALSE(task->done());
  EXPECT_TRUE(task->message().status.empty());

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SetTaskFailed>("SetTaskFailed");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree>)"
    R"(<SetTaskFailed task="{task}" reason="follow_path_aborted"/>)"
    R"(</BehaviorTree></root>)",
    makeBlackboard(task));

  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);

  // Advanced (done) but flagged failed via a non-empty structured status.
  EXPECT_TRUE(task->done());
  ASSERT_FALSE(task->message().status.empty());
  YAML::Node status = YAML::Load(task->message().status);
  EXPECT_EQ(status["outcome"].as<std::string>(), "failed");
  EXPECT_EQ(status["reason"].as<std::string>(), "follow_path_aborted");
}

TEST_F(SetTaskFailedTest, OmitsUnsetReason)
{
  auto task = makeTask("line_2");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SetTaskFailed>("SetTaskFailed");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree>)"
    R"(<SetTaskFailed task="{task}"/>)"
    R"(</BehaviorTree></root>)",
    makeBlackboard(task));

  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  YAML::Node status = YAML::Load(task->message().status);
  EXPECT_EQ(status["outcome"].as<std::string>(), "failed");
  EXPECT_FALSE(status["reason"]);   // unset reason is not written
}

TEST_F(SetTaskFailedTest, NullTaskSucceedsWithoutDereference)
{
  // The dispatch catchall can tick SetTaskFailed/SetTaskDone with a null current_task
  // (e.g. mission cleared / all done). It must not dereference the null pointer.
  auto bb = BT::Blackboard::create();
  bb->set("node", node_);
  bb->set("task", marine_nav_tasks::TaskPtr{});  // null

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SetTaskFailed>("SetTaskFailed");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree>)"
    R"(<SetTaskFailed task="{task}"/>)"
    R"(</BehaviorTree></root>)",
    bb);

  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

TEST_F(SetTaskFailedTest, DistinctFromSetTaskDone)
{
  // A clean SetTaskDone marks the task done but leaves status empty — the
  // exact distinction the coverage record relies on.
  auto task = makeTask("line_3");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SetTaskDone>("SetTaskDone");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree>)"
    R"(<SetTaskDone task="{task}"/>)"
    R"(</BehaviorTree></root>)",
    makeBlackboard(task));

  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(task->done());
  EXPECT_TRUE(task->message().status.empty());
}
