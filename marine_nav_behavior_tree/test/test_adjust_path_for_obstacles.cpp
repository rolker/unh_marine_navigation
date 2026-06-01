// Unit tests for the pure corridor-solver core of AdjustPathForObstacles:
// makeLateralOffsets, resampleStations, and solveCorridorOffsets. No ROS — the
// DP is a free function over a hand-built cost matrix, exercising the three
// canonical cases the plan calls out: clear -> nominal, blob -> bounded detour
// that re-anchors, wall -> infeasible (caller passes nominal through).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "marine_nav_behavior_tree/plugins/action/adjust_path_for_obstacles.h"

using marine_nav_behavior_tree::applyAvoidanceSlowdown;
using marine_nav_behavior_tree::CorridorParams;
using marine_nav_behavior_tree::makeLateralOffsets;
using marine_nav_behavior_tree::resampleStations;
using marine_nav_behavior_tree::solveCorridorOffsets;

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y, const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  return p;
}

double peakAbs(const std::vector<double> & v)
{
  double m = 0.0;
  for (double d : v) {
    m = std::max(m, std::abs(d));
  }
  return m;
}

}  // namespace

// --- makeLateralOffsets ----------------------------------------------------

TEST(MakeLateralOffsets, SymmetricWithExactZeroCentre)
{
  const auto o = makeLateralOffsets(2.0, 0.5);
  ASSERT_EQ(o.size(), 9u);            // -2 .. 2 step 0.5
  EXPECT_DOUBLE_EQ(o.front(), -2.0);
  EXPECT_DOUBLE_EQ(o.back(), 2.0);
  EXPECT_DOUBLE_EQ(o[4], 0.0);        // exact centre
}

TEST(MakeLateralOffsets, DegenerateStepYieldsSingleZero)
{
  const auto o = makeLateralOffsets(2.0, 0.0);
  ASSERT_EQ(o.size(), 1u);
  EXPECT_DOUBLE_EQ(o[0], 0.0);
}

TEST(MakeLateralOffsets, NeverExceedsMaxXteForNonMultiple)
{
  // max_xte not an exact multiple of the step: floor, so no offset overshoots
  // the corridor half-width (1.0 / 0.6 -> {-0.6, 0, 0.6}, NOT ±1.2).
  const auto o = makeLateralOffsets(1.0, 0.6);
  ASSERT_EQ(o.size(), 3u);
  for (double d : o) {
    EXPECT_LE(std::abs(d), 1.0 + 1e-9);
  }
  EXPECT_DOUBLE_EQ(o[1], 0.0);

  // Corridor narrower than one step -> only the centre (no deviation possible).
  const auto narrow = makeLateralOffsets(0.3, 0.5);
  ASSERT_EQ(narrow.size(), 1u);
  EXPECT_DOUBLE_EQ(narrow[0], 0.0);
}

// --- resampleStations ------------------------------------------------------

TEST(ResampleStations, StraightLineSpacingNormalAndYaw)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses = {makePose(0.0, 0.0), makePose(10.0, 0.0)};
  const auto st = resampleStations(poses, 2.0);
  ASSERT_GE(st.size(), 2u);
  EXPECT_NEAR(st.front().x, 0.0, 1e-9);
  EXPECT_NEAR(st.back().x, 10.0, 1e-9);   // endpoint always anchored
  // Tangent +x -> left normal (0, 1), yaw 0.
  EXPECT_NEAR(st.front().nx, 0.0, 1e-9);
  EXPECT_NEAR(st.front().ny, 1.0, 1e-9);
  EXPECT_NEAR(st.front().yaw, 0.0, 1e-9);
}

TEST(ResampleStations, FewerThanTwoPosesYieldsEmpty)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses = {makePose(0.0, 0.0)};
  EXPECT_TRUE(resampleStations(poses, 2.0).empty());
}

// --- solveCorridorOffsets --------------------------------------------------

namespace
{
// 7 stations, 9 offsets; whole line active (ends auto-anchored to 0).
constexpr std::size_t kN = 7;
const std::vector<double> kOffsets = makeLateralOffsets(2.0, 0.5);

CorridorParams defaultParams()
{
  CorridorParams p;
  p.max_xte = 2.0;
  p.lateral_step = 0.5;
  p.w_xte = 1.0;
  p.w_obs = 0.02;
  p.w_smooth = 0.1;
  p.w_temporal = 0.0;
  p.max_lateral_rate = 1.0;
  p.lethal_cost = 253.0;
  return p;
}
}  // namespace

TEST(SolveCorridorOffsets, ClearWaterStaysOnLine)
{
  std::vector<std::vector<double>> costs(kN, std::vector<double>(kOffsets.size(), 0.0));
  const auto r = solveCorridorOffsets(costs, kOffsets, defaultParams(), {}, 0, kN);
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(peakAbs(*r), 0.0, 1e-9);   // no obstacle, no deviation
}

TEST(SolveCorridorOffsets, BlobProducesBoundedDetourThatReanchors)
{
  std::vector<std::vector<double>> costs(kN, std::vector<double>(kOffsets.size(), 0.0));
  // Block the centre columns (|d| < 1.0 -> indices 3,4,5) at interior stations
  // 2,3,4, forcing the path off-centre there.
  for (std::size_t i = 2; i <= 4; ++i) {
    for (std::size_t j = 3; j <= 5; ++j) {
      costs[i][j] = 254.0;  // lethal
    }
  }
  const auto r = solveCorridorOffsets(costs, kOffsets, defaultParams(), {}, 0, kN);
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(r->front(), 0.0, 1e-9);            // anchored start
  EXPECT_NEAR(r->back(), 0.0, 1e-9);             // anchored end
  EXPECT_GE(peakAbs(*r), 1.0 - 1e-9);            // deviated clear of the blob
  EXPECT_LE(peakAbs(*r), 2.0 + 1e-9);            // within the corridor
}

TEST(SolveCorridorOffsets, FullWallIsInfeasible)
{
  std::vector<std::vector<double>> costs(kN, std::vector<double>(kOffsets.size(), 0.0));
  for (std::size_t j = 0; j < kOffsets.size(); ++j) {
    costs[3][j] = 254.0;   // an entire station blocked across the corridor
  }
  const auto r = solveCorridorOffsets(costs, kOffsets, defaultParams(), {}, 0, kN);
  EXPECT_FALSE(r.has_value());
}

TEST(SolveCorridorOffsets, LethalAnchorIsInfeasible)
{
  // A lethal cell on the centreline at an anchor endpoint (here the start
  // anchor, station 0, centre column 4) must make the corridor infeasible —
  // a pinned station is forced to d=0, so it cannot route around the obstacle,
  // and the solver must not report a clear path through it.
  std::vector<std::vector<double>> costs(kN, std::vector<double>(kOffsets.size(), 0.0));
  costs[0][4] = 254.0;  // lethal at the start-anchor's d=0 cell
  const auto r = solveCorridorOffsets(costs, kOffsets, defaultParams(), {}, 0, kN);
  EXPECT_FALSE(r.has_value());

  // Same at the end anchor (station kN-1).
  std::vector<std::vector<double>> costs_end(kN, std::vector<double>(kOffsets.size(), 0.0));
  costs_end[kN - 1][4] = 254.0;
  const auto r_end = solveCorridorOffsets(costs_end, kOffsets, defaultParams(), {}, 0, kN);
  EXPECT_FALSE(r_end.has_value());
}

TEST(SolveCorridorOffsets, RespectsLateralRateLimit)
{
  std::vector<std::vector<double>> costs(kN, std::vector<double>(kOffsets.size(), 0.0));
  for (std::size_t i = 2; i <= 4; ++i) {
    for (std::size_t j = 3; j <= 5; ++j) {
      costs[i][j] = 254.0;
    }
  }
  const auto r = solveCorridorOffsets(costs, kOffsets, defaultParams(), {}, 0, kN);
  ASSERT_TRUE(r.has_value());
  for (std::size_t i = 1; i < r->size(); ++i) {
    EXPECT_LE(std::abs((*r)[i] - (*r)[i - 1]), 1.0 + 1e-9);  // max_lateral_rate
  }
}

// --- applyAvoidanceSlowdown ------------------------------------------------

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
