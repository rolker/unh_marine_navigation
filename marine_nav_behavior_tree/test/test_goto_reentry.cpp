// Regression tests for the #52 goto re-command fix.
//
// GotoTask wraps its inner Sequence in RestartOnTaskChange (mirroring the hover
// fix #46). The dispatch Switch keys on task *type*, so re-issuing a goto override
// (still type 'goto') never halts the branch on its own; without the decorator the
// Sequence stays parked at the RUNNING GotoPose child, SetPathFromTask is never
// re-ticked, and the boat keeps driving to the original target.
//
// Goto differs from hover in one way that matters here: its final child
// (GotoPose / NavigateThroughWaypoints) DOES return SUCCESS when the target is
// reached, whereas Hover holds forever. These tests pin both the re-command
// behaviour and that the decorator does not interfere with normal completion.
//
// Pure BT-mechanics — no ROS fixture. A counting leaf stands in for
// SetPathFromTask (the per-entry target read); a RUNNING-then-SUCCESS leaf stands
// in for GotoPose driving to and reaching the target.

#include <gtest/gtest.h>

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "marine_nav_behavior_tree/plugins/decorator/restart_on_task_change.h"
#include "rclcpp/time.hpp"

namespace
{

// Counts ticks. Stands in for SetPathFromTask — the work that must re-run on each
// goto (re-)entry so the new target is read.
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

// RUNNING for a fixed number of ticks, then SUCCESS. Stands in for GotoPose
// driving to the target and arriving (unlike Hover, which never succeeds).
class ArrivingLeaf : public BT::StatefulActionNode
{
public:
  ArrivingLeaf(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  static int ticks_until_arrival;
  BT::NodeStatus onStart() override
  {
    remaining_ = ticks_until_arrival;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (remaining_-- <= 0) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}

private:
  int remaining_{0};
};
int ArrivingLeaf::ticks_until_arrival = 3;

BT::BehaviorTreeFactory makeFactory()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CountingAction>("Counter");
  factory.registerNodeType<ArrivingLeaf>("ArrivingLeaf");
  factory.registerNodeType<marine_nav_behavior_tree::RestartOnTaskChange>("RestartOnTaskChange");
  return factory;
}

// Mirrors GotoTask's inner shape: RestartOnTaskChange[ Sequence[ SetPathFromTask,
// GotoPose, SetTaskDone ] ]. SetTaskDone is a no-op Counter-free SUCCESS leaf here.
const char * kGotoShape =
  R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
       <RestartOnTaskChange task_update_time="{t}">
         <Sequence><Counter/><ArrivingLeaf/></Sequence>
       </RestartOnTaskChange>
     </BehaviorTree></root>)";

const rclcpp::Time T1(10, 0, RCL_ROS_TIME);
const rclcpp::Time T2(20, 0, RCL_ROS_TIME);

}  // namespace

// A re-issued goto (timestamp bump) while GotoPose is RUNNING re-ticks
// SetPathFromTask, so the new target is read.
TEST(GotoReentry, ReissuedGotoRereadsTarget)
{
  CountingAction::count = 0;
  ArrivingLeaf::ticks_until_arrival = 5;
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(kGotoShape);

  tree.rootBlackboard()->set("t", T1);
  tree.tickOnce();                      // baseline T1; Counter (1); ArrivingLeaf RUNNING
  EXPECT_EQ(CountingAction::count, 1);

  tree.tickOnce();                      // still driving, t stable → no re-read
  EXPECT_EQ(CountingAction::count, 1);

  tree.rootBlackboard()->set("t", T2);
  tree.tickOnce();                      // re-issued goto → restart; Counter (2)
  EXPECT_EQ(CountingAction::count, 2);
}

// While the timestamp is stable, the drive to the target is not disturbed: the
// target is read once and the goto runs to completion.
TEST(GotoReentry, StableGotoRunsToCompletionWithoutReread)
{
  CountingAction::count = 0;
  ArrivingLeaf::ticks_until_arrival = 3;
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(kGotoShape);

  tree.rootBlackboard()->set("t", T1);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 10 && status == BT::NodeStatus::RUNNING; ++i) {
    status = tree.tickOnce();
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);   // goto completes normally
  EXPECT_EQ(CountingAction::count, 1);          // target read exactly once
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
