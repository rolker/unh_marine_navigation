// Unit tests for SetPathFromTask::buildPath — the static helper that copies
// a pose index-range into a Path. Guards the TF-extrapolation fix from #23:
// the outer header.stamp must be zero ("latest" in TF lookups), the outer
// frame_id round-trips, and per-pose stamps stay untouched (downstream
// crabbing_path_follower and marine_nav_utilities rely on them).

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "marine_nav_behavior_tree/plugins/action/set_path_from_task.h"
#include "nav_msgs/msg/path.hpp"

using marine_nav_behavior_tree::SetPathFromTask;

namespace
{

geometry_msgs::msg::PoseStamped makePose(
  double x, int32_t sec, uint32_t nanosec, const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.header.stamp.sec = sec;
  p.header.stamp.nanosec = nanosec;
  p.pose.position.x = x;
  return p;
}

// Three poses with deliberately stale, distinct stamps — the canonical
// "task uploaded minutes ago, now line 3 of the survey" shape.
std::vector<geometry_msgs::msg::PoseStamped> stalePoses()
{
  return {
    makePose(0.0, 1000, 111),
    makePose(1.0, 1001, 222),
    makePose(2.0, 1002, 333),
  };
}

}  // namespace

// Core fix: outer stamp must be zero ("latest" in TF lookups) — not the stale
// first-pose stamp the legacy code copied.
TEST(SetPathFromTaskBuildPath, ZeroesOuterStampOnNonEmptyResult)
{
  auto path = SetPathFromTask::buildPath(stalePoses(), 0, -1);
  EXPECT_EQ(path.header.stamp.sec, 0);
  EXPECT_EQ(path.header.stamp.nanosec, 0u);
}

// Spatial reference is real and must round-trip — only the stamp is "latest".
TEST(SetPathFromTaskBuildPath, PreservesOuterFrameId)
{
  auto poses = stalePoses();
  for(auto & p : poses) {
    p.header.frame_id = "odom";
  }
  auto path = SetPathFromTask::buildPath(poses, 0, -1);
  EXPECT_EQ(path.header.frame_id, "odom");
}

// Per-pose stamps are load-bearing downstream (crabbing segment timing,
// utilities trajectory computation). The fix must NOT zero them.
TEST(SetPathFromTaskBuildPath, PerPoseStampsUntouched)
{
  auto path = SetPathFromTask::buildPath(stalePoses(), 0, -1);
  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_EQ(path.poses[0].header.stamp.sec, 1000);
  EXPECT_EQ(path.poses[0].header.stamp.nanosec, 111u);
  EXPECT_EQ(path.poses[1].header.stamp.sec, 1001);
  EXPECT_EQ(path.poses[2].header.stamp.sec, 1002);
}

// Negative end_index counts from the end — preserve original behaviour.
TEST(SetPathFromTaskBuildPath, NegativeEndIndexNormalizes)
{
  EXPECT_EQ(SetPathFromTask::buildPath(stalePoses(), 0, -1).poses.size(), 3u);
  EXPECT_EQ(SetPathFromTask::buildPath(stalePoses(), 0, -2).poses.size(), 2u);
}

// Sub-range [1, 2] selects the middle and last pose.
TEST(SetPathFromTaskBuildPath, SubRangeSelectsCorrectly)
{
  auto path = SetPathFromTask::buildPath(stalePoses(), 1, 2);
  ASSERT_EQ(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 2.0);
}

// Single-pose range [start == end] returns exactly one pose.
TEST(SetPathFromTaskBuildPath, SinglePoseRange)
{
  auto path = SetPathFromTask::buildPath(stalePoses(), 1, 1);
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(path.poses[0].header.stamp.sec, 1001);   // per-pose preserved
  EXPECT_EQ(path.header.stamp.sec, 0);               // outer zeroed
}

// Empty input → default-constructed Path: the `if(!path.poses.empty())`
// guard skips the header assignment, so frame_id stays empty and stamp zero.
TEST(SetPathFromTaskBuildPath, EmptyInputReturnsZeroHeaderPath)
{
  auto path = SetPathFromTask::buildPath({}, 0, -1);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.stamp.sec, 0);
  EXPECT_EQ(path.header.stamp.nanosec, 0u);
  EXPECT_TRUE(path.header.frame_id.empty());
}

// Inverted range (start > end) returns an empty path with a default header.
TEST(SetPathFromTaskBuildPath, InvertedRangeReturnsEmpty)
{
  auto path = SetPathFromTask::buildPath(stalePoses(), 2, 1);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.stamp.sec, 0);
  EXPECT_TRUE(path.header.frame_id.empty());
}

// Very-negative end_index (more negative than -size) leaves it still negative
// after the "counts from end" normalization. The bounds guard must catch this
// and return empty, not silently wrap to size_t and copy the whole vector.
TEST(SetPathFromTaskBuildPath, VeryNegativeEndIndexReturnsEmpty)
{
  auto path = SetPathFromTask::buildPath(stalePoses(), 0, -5);   // size=3, -5 → -2
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.stamp.sec, 0);
}

// Negative start_index would also wrap to a huge size_t in the loop init.
// Treat any negative index as out-of-range → empty path.
TEST(SetPathFromTaskBuildPath, NegativeStartIndexReturnsEmpty)
{
  auto path = SetPathFromTask::buildPath(stalePoses(), -1, 2);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.stamp.sec, 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
