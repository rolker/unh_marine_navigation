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
using marine_nav_utilities::planCorridorOffsets;
using marine_nav_utilities::resampleStations;
using marine_nav_utilities::solveCorridorOffsets;
using marine_nav_utilities::Station;

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

// --- planCorridorOffsets (node-free core of the controller avoider) --------

namespace
{
// A straight survey line along +x at 1 m spacing, left-normal (0, 1).
std::vector<Station> straightStations(std::size_t n)
{
  std::vector<Station> st;
  for (std::size_t i = 0; i < n; ++i) {
    Station s;
    s.x = static_cast<double>(i);
    s.y = 0.0;
    s.nx = 0.0;
    s.ny = 1.0;
    s.yaw = 0.0;
    st.push_back(s);
  }
  return st;
}

CorridorParams planParams()
{
  CorridorParams p = defaultParams();
  p.max_xte = 3.0;        // wider corridor so the detour fits
  p.w_smooth = 0.1;
  return p;
}
}  // namespace

TEST(PlanCorridorOffsets, ClearWaterReturnsNoDeviation)
{
  const auto stations = straightStations(20);
  auto sample = [](double, double) { return 0.0; };  // all free
  const auto r = planCorridorOffsets(
    stations, sample, planParams(), 1.0, 4.0, /*robot*/ 2.0, 0.0, {});
  ASSERT_TRUE(r.has_value());
  for (double d : *r) {
    EXPECT_NEAR(d, 0.0, 1e-9);
  }
}

TEST(PlanCorridorOffsets, BendsAroundAnObstacleAheadAndReanchors)
{
  const auto stations = straightStations(20);
  // Lethal blob straddling the line near x in [9,11], |y| < 0.75.
  auto sample = [](double x, double y) -> double {
    if (x >= 9.0 && x <= 11.0 && std::abs(y) < 0.75) {
      return 254.0;
    }
    return 0.0;
  };
  const auto r = planCorridorOffsets(
    stations, sample, planParams(), 1.0, 4.0, /*robot*/ 2.0, 0.0, {});
  ASSERT_TRUE(r.has_value());
  double peak = 0.0;
  for (double d : *r) {
    peak = std::max(peak, std::abs(d));
  }
  EXPECT_GE(peak, 0.75);                  // deviated clear of the blob
  EXPECT_LE(peak, 3.0 + 1e-9);            // within the corridor
  EXPECT_NEAR(r->back(), 0.0, 1e-9);      // re-anchors to the line at the end
}

TEST(PlanCorridorOffsets, AnchorBehindLetsTheBoatDeviateWherePinAtBoatCannot)
{
  // The #59 fix, demonstrated by contrast. An obstacle straddles the boat's own
  // position (x in [8,10], boat at x=9). With anchor_behind=0 the detour's near
  // anchor is pinned to d=0 *at the boat* (the old BT-node behaviour) — which
  // lands the anchor on a lethal cell, so no corridor exists (nullopt). With
  // anchor_behind>0 the anchor sits behind the boat, leaving the boat free to
  // carry a non-zero offset and clear the obstacle.
  const auto stations = straightStations(24);
  auto sample = [](double x, double y) -> double {
    if (x >= 8.0 && x <= 10.0 && std::abs(y) < 0.75) {
      return 254.0;
    }
    return 0.0;
  };
  const double robot_x = 9.0;

  const auto pinned_at_boat = planCorridorOffsets(
    stations, sample, planParams(), 1.0, /*anchor_behind*/ 0.0, robot_x, 0.0, {});
  EXPECT_FALSE(pinned_at_boat.has_value());  // boat pinned onto the obstacle

  const auto anchored_behind = planCorridorOffsets(
    stations, sample, planParams(), 1.0, /*anchor_behind*/ 4.0, robot_x, 0.0, {});
  ASSERT_TRUE(anchored_behind.has_value());
  EXPECT_GT(std::abs((*anchored_behind)[9]), 1e-6);  // boat free to deviate
  EXPECT_NEAR(anchored_behind->front(), 0.0, 1e-9);  // still re-anchors behind
}

TEST(PlanCorridorOffsets, OutOfWindowReturnsNullopt)
{
  const auto stations = straightStations(20);
  auto sample = [](double, double) { return -1.0; };  // entire line out of window
  const auto r = planCorridorOffsets(
    stations, sample, planParams(), 1.0, 4.0, 2.0, 0.0, {});
  EXPECT_FALSE(r.has_value());
}

TEST(PlanCorridorOffsets, FullWallAheadIsInfeasible)
{
  const auto stations = straightStations(20);
  // A wall across the whole corridor at x in [9,10] (all |y| lethal).
  auto sample = [](double x, double) -> double {
    return (x >= 9.0 && x <= 10.0) ? 254.0 : 0.0;
  };
  const auto r = planCorridorOffsets(
    stations, sample, planParams(), 1.0, 4.0, 2.0, 0.0, {});
  EXPECT_FALSE(r.has_value());
}
