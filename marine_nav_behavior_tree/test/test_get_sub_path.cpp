// Unit tests for GetSubPath::buildSubPath — the static helper that extracts
// a pose index-range from an input Path. Same fix shape as
// SetPathFromTask::buildPath; defense-in-depth for #23 (this producer builds
// {transit_path}, whose stale stamp is dropped at PathToPoseVector before
// reaching Nav2 FollowPath, but the identical bug shape is fixed for
// regression resistance against future BT rewiring).

#include <gtest/gtest.h>

#include <cstdint>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "marine_nav_behavior_tree/plugins/action/get_sub_path.h"
#include "nav_msgs/msg/path.hpp"

using marine_nav_behavior_tree::GetSubPath;

namespace
{

// Input path with a stale outer stamp AND distinct per-pose stamps — exercises
// both: the outer-stamp zeroing fix and the per-pose preservation invariant.
nav_msgs::msg::Path makeStalePath()
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.header.stamp.sec = 999;   // outer (stale, what the bug propagated)

  for(int i = 0; i < 3; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp.sec = 1000 + i;
    pose.header.stamp.nanosec = static_cast<uint32_t>(111 * (i + 1));
    pose.pose.position.x = static_cast<double>(i);
    p.poses.push_back(pose);
  }
  return p;
}

}  // namespace

// Core fix: outer stamp must be zero ("latest" in TF lookups), not the input
// path's outer stamp nor the first per-pose stamp.
TEST(GetSubPathBuildSubPath, ZeroesOuterStampOnNonEmptyResult)
{
  auto path = GetSubPath::buildSubPath(makeStalePath(), 0, -1);
  EXPECT_EQ(path.header.stamp.sec, 0);
  EXPECT_EQ(path.header.stamp.nanosec, 0u);
}

// Outer frame_id round-trips (spatial reference, not stamp).
TEST(GetSubPathBuildSubPath, PreservesOuterFrameId)
{
  auto in = makeStalePath();
  for(auto& p : in.poses) p.header.frame_id = "odom";
  auto path = GetSubPath::buildSubPath(in, 0, -1);
  EXPECT_EQ(path.header.frame_id, "odom");
}

// Per-pose stamps stay untouched — load-bearing downstream.
TEST(GetSubPathBuildSubPath, PerPoseStampsUntouched)
{
  auto path = GetSubPath::buildSubPath(makeStalePath(), 0, -1);
  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_EQ(path.poses[0].header.stamp.sec, 1000);
  EXPECT_EQ(path.poses[0].header.stamp.nanosec, 111u);
  EXPECT_EQ(path.poses[1].header.stamp.sec, 1001);
  EXPECT_EQ(path.poses[2].header.stamp.sec, 1002);
}

TEST(GetSubPathBuildSubPath, NegativeEndIndexNormalizes)
{
  EXPECT_EQ(GetSubPath::buildSubPath(makeStalePath(), 0, -1).poses.size(), 3u);
  EXPECT_EQ(GetSubPath::buildSubPath(makeStalePath(), 0, -2).poses.size(), 2u);
}

TEST(GetSubPathBuildSubPath, SubRangeSelectsCorrectly)
{
  auto path = GetSubPath::buildSubPath(makeStalePath(), 1, 2);
  ASSERT_EQ(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 2.0);
}

TEST(GetSubPathBuildSubPath, SinglePoseRange)
{
  auto path = GetSubPath::buildSubPath(makeStalePath(), 1, 1);
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(path.poses[0].header.stamp.sec, 1001);
  EXPECT_EQ(path.header.stamp.sec, 0);
}

// Empty input path → default-constructed Path (the empty-result guard skips
// the header assignment).
TEST(GetSubPathBuildSubPath, EmptyInputReturnsZeroHeaderPath)
{
  nav_msgs::msg::Path empty;
  auto path = GetSubPath::buildSubPath(empty, 0, -1);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.stamp.sec, 0);
  EXPECT_TRUE(path.header.frame_id.empty());
}

TEST(GetSubPathBuildSubPath, InvertedRangeReturnsEmpty)
{
  auto path = GetSubPath::buildSubPath(makeStalePath(), 2, 1);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.stamp.sec, 0);
  EXPECT_TRUE(path.header.frame_id.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
