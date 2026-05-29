// Regression tests for the #46 hover-target latch fix.
//
// Two things are pinned here:
//
//  1. The BT-mechanics contract the HoverTask fix depends on: a plain `Sequence`
//     re-ticks its first child after a halt/re-entry, whereas `SequenceWithMemory`
//     does NOT (it keeps its child index). Because the real `Hover` action never
//     returns SUCCESS, SequenceWithMemory never reset and so never re-ran
//     PredictStoppingPose — freezing the hover target at the first value ever
//     computed. The same test also pins the no-regression direction: a *continuous*
//     hold (tick → tick, no halt) must NOT re-tick the first child (recomputing the
//     stop point every tick would chase the drifting boat).
//
//  2. The `RestartOnTaskChange` decorator: it forces a child re-entry when the
//     watched timestamp changes (a same-type hover re-command, which the dispatch
//     Switch does not halt on), and leaves the child untouched while the timestamp
//     is stable (no thrash on benign feedback re-sends).
//
// Pure BT-mechanics — no ROS fixture. A counting leaf stands in for
// "Fallback + PredictStoppingPose" and a perpetually-RUNNING leaf for `Hover`.

#include <gtest/gtest.h>

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "marine_nav_behavior_tree/plugins/decorator/restart_on_task_change.h"
#include "rclcpp/time.hpp"

namespace
{

// Counts how many times it is ticked. Stands in for the work that must re-run on
// each hover entry (goal-set Fallback + PredictStoppingPose).
class CountingAction : public BT::SyncActionNode
{
public:
  CountingAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  static int count;

  BT::NodeStatus tick() override
  {
    ++count;
    return BT::NodeStatus::SUCCESS;
  }
};
int CountingAction::count = 0;

// Always RUNNING, cleanly haltable. Stands in for the perpetual-station-keep
// `Hover` action that never returns SUCCESS.
class RunningLeaf : public BT::StatefulActionNode
{
public:
  RunningLeaf(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
};

BT::BehaviorTreeFactory makeFactory()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CountingAction>("Counter");
  factory.registerNodeType<RunningLeaf>("RunningLeaf");
  factory.registerNodeType<marine_nav_behavior_tree::RestartOnTaskChange>(
    "RestartOnTaskChange");
  return factory;
}

const rclcpp::Time T1(10, 0, RCL_ROS_TIME);
const rclcpp::Time T2(20, 0, RCL_ROS_TIME);

}  // namespace

// --- 1. BT-mechanics contract -------------------------------------------------

// Plain Sequence re-ticks the first child after a halt → re-entry recomputes.
// This is the behaviour the HoverTask fix relies on.
TEST(HoverReentry, PlainSequenceReticksFirstChildAfterHalt)
{
  CountingAction::count = 0;
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
         <Sequence><Counter/><RunningLeaf/></Sequence>
       </BehaviorTree></root>)");

  tree.tickOnce();                       // Counter (1), RunningLeaf RUNNING
  EXPECT_EQ(CountingAction::count, 1);
  tree.haltTree();                       // plain Sequence resets to child 0
  tree.tickOnce();                       // Counter re-ticked (2)
  EXPECT_EQ(CountingAction::count, 2);
}

// SequenceWithMemory does NOT re-tick after a halt — documents the bug: with the
// never-succeeding RunningLeaf the node never resets, so the Counter runs once.
TEST(HoverReentry, SequenceWithMemoryDoesNotRetickAfterHalt)
{
  CountingAction::count = 0;
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
         <SequenceWithMemory><Counter/><RunningLeaf/></SequenceWithMemory>
       </BehaviorTree></root>)");

  tree.tickOnce();
  EXPECT_EQ(CountingAction::count, 1);
  tree.haltTree();                       // SequenceWithMemory KEEPS its index
  tree.tickOnce();                       // resumes at RunningLeaf; Counter skipped
  EXPECT_EQ(CountingAction::count, 1);
}

// Continuous hold (no halt) must NOT re-tick the first child — guards against a
// regression where the stop point would be recomputed every tick.
TEST(HoverReentry, PlainSequenceDoesNotRetickDuringContinuousHold)
{
  CountingAction::count = 0;
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
         <Sequence><Counter/><RunningLeaf/></Sequence>
       </BehaviorTree></root>)");

  for (int i = 0; i < 5; ++i) {
    tree.tickOnce();                     // no halt between ticks
  }
  EXPECT_EQ(CountingAction::count, 1);   // resumed at RunningLeaf each time
}

// --- 2. RestartOnTaskChange decorator ----------------------------------------

// A changed timestamp forces the child to re-enter (Counter re-runs); a stable
// timestamp does not.
TEST(RestartOnTaskChange, RestartsWhenTimestampChanges)
{
  CountingAction::count = 0;
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
         <RestartOnTaskChange task_update_time="{t}">
           <Sequence><Counter/><RunningLeaf/></Sequence>
         </RestartOnTaskChange>
       </BehaviorTree></root>)");

  tree.rootBlackboard()->set("t", T1);
  tree.tickOnce();                       // baseline T1; Counter (1)
  EXPECT_EQ(CountingAction::count, 1);

  tree.tickOnce();                       // t unchanged → no restart
  EXPECT_EQ(CountingAction::count, 1);

  tree.rootBlackboard()->set("t", T2);
  tree.tickOnce();                       // t changed → restart; Counter (2)
  EXPECT_EQ(CountingAction::count, 2);
}

TEST(RestartOnTaskChange, NoRestartWhileTimestampStable)
{
  CountingAction::count = 0;
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
         <RestartOnTaskChange task_update_time="{t}">
           <Sequence><Counter/><RunningLeaf/></Sequence>
         </RestartOnTaskChange>
       </BehaviorTree></root>)");

  tree.rootBlackboard()->set("t", T1);
  for (int i = 0; i < 5; ++i) {
    tree.tickOnce();
  }
  EXPECT_EQ(CountingAction::count, 1);   // never restarted
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
