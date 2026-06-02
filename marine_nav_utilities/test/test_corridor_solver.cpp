// Unit tests for the pure corridor solver: makeLateralOffsets, resampleStations,
// and solveCorridorOffsets. No ROS — the DP is a free function over a hand-built
// cost matrix, exercising the canonical cases: clear -> nominal, blob -> bounded
// detour that re-anchors, wall -> infeasible (caller passes nominal through).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "marine_nav_utilities/corridor_solver.h"

using marine_nav_utilities::CorridorParams;
using marine_nav_utilities::makeLateralOffsets;
using marine_nav_utilities::resampleStations;
using marine_nav_utilities::solveCorridorOffsets;

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
