// Unit tests for applyAvoidanceSlowdown — the per-pose-stamp speed shaping the
// AdjustPathForObstacles node applies to the deviating run. The pure corridor
// solver (makeLateralOffsets, resampleStations, solveCorridorOffsets) now lives
// in marine_nav_utilities and is tested in test_corridor_solver.cpp (#59).

#include <gtest/gtest.h>

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "marine_nav_behavior_tree/plugins/action/adjust_path_for_obstacles.h"

using marine_nav_behavior_tree::applyAvoidanceSlowdown;

namespace
{
nav_msgs::msg::Path straightPath(int n)
{
  // n poses 1 m apart along +x; stamps left zero.
  nav_msgs::msg::Path path;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = i * 1.0;
    path.poses.push_back(p);
  }
  return path;
}

double stampSeconds(const geometry_msgs::msg::PoseStamped & p)
{
  return p.header.stamp.sec + p.header.stamp.nanosec * 1e-9;
}
}  // namespace

TEST(ApplyAvoidanceSlowdown, StampsDeviatingRunAtTargetSpeed)
{
  auto path = straightPath(7);
  std::vector<double> d(7, 0.0);
  d[2] = 1.0;
  d[3] = 1.5;
  d[4] = 1.0;  // deviating run [2,4]

  applyAvoidanceSlowdown(path, d, 0.5, 0.05);  // 0.5 m/s through the run

  // Outside the run: untouched (zero) stamps.
  for (int i : {0, 1, 5, 6}) {
    EXPECT_EQ(path.poses[i].header.stamp.sec, 0);
    EXPECT_EQ(path.poses[i].header.stamp.nanosec, 0u);
  }
  // In the run: non-zero, strictly increasing, implied speed == 0.5 m/s.
  EXPECT_GT(stampSeconds(path.poses[2]), 0.0);
  for (int i = 3; i <= 4; ++i) {
    const double dt = stampSeconds(path.poses[i]) - stampSeconds(path.poses[i - 1]);
    const double dist = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    EXPECT_GT(dt, 0.0);
    EXPECT_NEAR(dist / dt, 0.5, 1e-6);
  }
}

TEST(ApplyAvoidanceSlowdown, DisabledOrNoRunLeavesStampsZero)
{
  auto path = straightPath(5);
  std::vector<double> d(5, 0.0);
  d[2] = 1.0;  // a deviating pose, but...
  applyAvoidanceSlowdown(path, d, 0.0, 0.05);  // ...avoid_speed = 0 disables
  for (const auto & p : path.poses) {
    EXPECT_EQ(p.header.stamp.sec, 0);
    EXPECT_EQ(p.header.stamp.nanosec, 0u);
  }

  // Single deviating pose => no 2-pose segment => no-op even when enabled.
  auto path2 = straightPath(5);
  std::vector<double> single(5, 0.0);
  single[2] = 1.0;
  applyAvoidanceSlowdown(path2, single, 0.5, 0.05);
  for (const auto & p : path2.poses) {
    EXPECT_EQ(p.header.stamp.sec, 0);
    EXPECT_EQ(p.header.stamp.nanosec, 0u);
  }
}
