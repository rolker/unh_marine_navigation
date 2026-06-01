// Unit tests for RobotOnPath's geometry (#52).
//
// The tick() wraps a TF pose lookup around minDistanceToPath(); the geometry is
// factored out as a static method so it can be pinned here without a TF fixture.
// The point being made for the mid-line-resume fix: distance is to the *nearest
// segment* of the line, so a boat parked 60 m along a 100 m line reads as "on it"
// for a few-metre threshold even though it is far from the line's start.

#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include "marine_nav_behavior_tree/plugins/condition/robot_on_path.h"

using marine_nav_behavior_tree::RobotOnPath;

namespace
{
geometry_msgs::msg::Point pt(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & pts)
{
  nav_msgs::msg::Path path;
  for (const auto & xy : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = xy.first;
    ps.pose.position.y = xy.second;
    path.poses.push_back(ps);
  }
  return path;
}
}  // namespace

TEST(RobotOnPath, EmptyPathReturnsNegative)
{
  EXPECT_LT(RobotOnPath::minDistanceToPath(pt(0, 0), makePath({})), 0.0);
}

TEST(RobotOnPath, SinglePoseIsPointDistance)
{
  EXPECT_DOUBLE_EQ(RobotOnPath::minDistanceToPath(pt(0, 0), makePath({{3.0, 4.0}})), 5.0);
}

TEST(RobotOnPath, OnTheLineIsZero)
{
  EXPECT_NEAR(RobotOnPath::minDistanceToPath(pt(5, 0), makePath({{0, 0}, {10, 0}})), 0.0, 1e-9);
}

TEST(RobotOnPath, PerpendicularDistanceToSegment)
{
  EXPECT_NEAR(RobotOnPath::minDistanceToPath(pt(5, 3), makePath({{0, 0}, {10, 0}})), 3.0, 1e-9);
}

TEST(RobotOnPath, BeyondSegmentEndClampsToVertex)
{
  // Point past the far end of the only segment → distance to the (10, 0) vertex.
  EXPECT_NEAR(RobotOnPath::minDistanceToPath(pt(13, 4), makePath({{0, 0}, {10, 0}})), 5.0, 1e-9);
}

TEST(RobotOnPath, NearestSegmentOfMultiSegmentLine)
{
  // L-shaped line; point is closest to the second segment.
  auto path = makePath({{0, 0}, {10, 0}, {10, 10}});
  EXPECT_NEAR(RobotOnPath::minDistanceToPath(pt(8, 5), path), 2.0, 1e-9);
}

TEST(RobotOnPath, MidLineResumeIsOnTheLine)
{
  // The fix's core case: boat ~60 m along a 100 m line, 2 m off it. "On the line"
  // for a few-metre threshold despite being 60 m from the start.
  auto path = makePath({{0, 0}, {100, 0}});
  EXPECT_NEAR(RobotOnPath::minDistanceToPath(pt(60, 2), path), 2.0, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
