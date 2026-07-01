#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using marine_nav_crabbing_path_follower::circumscribedRadius;
using marine_nav_crabbing_path_follower::curvatureSpeedFactor;
using marine_nav_crabbing_path_follower::lookaheadPoint;
using marine_nav_crabbing_path_follower::turnSpeedFactor;

namespace
{
Point makePoint(double x, double y)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

PoseStamped makePose(double x, double y)
{
  PoseStamped ps;
  ps.pose.position = makePoint(x, y);
  return ps;
}

// Mirrors the anticipatory-curvature composition at
// crabbing_path_follower.cpp:991-1006 as a pure function so the *wiring* (the
// along-track foot / half-lookahead / full-lookahead point selection and the
// min() with the reactive turn factor) can be exercised end-to-end over a real
// multi-segment global_plan_ without ROS lifecycle scaffolding. Any change to
// the fit-point selection or the min() direction at the call site must be kept
// in lockstep with this helper.
struct SpeedRegulation
{
  double turn_factor;
  double curvature_factor;
  double combined;
};

SpeedRegulation regulate(
  const std::vector<PoseStamped> & poses, int current_segment, double progress,
  double lookahead, double crab_angle_deg, double max_crab_deg, double min_factor,
  double curvature_min_radius)
{
  // The three PATH-REFERENCED fit points, exactly as the call site derives them.
  const Point foot = lookaheadPoint(poses, current_segment, progress, 0.0);
  const Point half = lookaheadPoint(poses, current_segment, progress, lookahead / 2.0);
  const Point full = lookaheadPoint(poses, current_segment, progress, lookahead);

  double curvature_factor = 1.0;
  if (lookahead > 0.0 && curvature_min_radius > 0.0) {
    const double radius = circumscribedRadius(foot, half, full);
    curvature_factor = curvatureSpeedFactor(radius, curvature_min_radius, min_factor);
  }
  const double turn_factor = turnSpeedFactor(crab_angle_deg, max_crab_deg, min_factor);
  return {turn_factor, curvature_factor, std::min(turn_factor, curvature_factor)};
}

// An L-shaped plan: a 20 m run east, a right-angle corner, then a run north.
// current_segment_ = 0, progress = 0 puts the boat at the very start.
std::vector<PoseStamped> lShapedPlan()
{
  return {makePose(0.0, 0.0), makePose(20.0, 0.0), makePose(20.0, 20.0), makePose(40.0, 20.0)};
}

// A straight three-vertex plan: no curvature anywhere along it.
std::vector<PoseStamped> straightPlan()
{
  return {makePose(0.0, 0.0), makePose(20.0, 0.0), makePose(40.0, 0.0)};
}
}  // namespace

// A straight run (three collinear points) has no curvature: the circumscribed
// radius is infinite, which maps to no slowdown (factor 1.0).
TEST(CircumscribedRadius, CollinearIsInfinite)
{
  const double r = circumscribedRadius(
    makePoint(0.0, 0.0), makePoint(1.0, 0.0), makePoint(2.0, 0.0));
  EXPECT_TRUE(std::isinf(r));
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(r, 50.0, 0.3), 1.0);
}

// A right triangle circumscribes a circle whose diameter is the hypotenuse, so
// its radius is exact and hand-checkable: legs 6 and 8, hypotenuse 10, R = 5.
// (R = |AB|·|BC|·|CA| / (4·Area) = 6·10·8 / (4·24) = 5.)
TEST(CircumscribedRadius, RightTriangleHasExactRadius)
{
  const double r = circumscribedRadius(
    makePoint(0.0, 0.0), makePoint(6.0, 0.0), makePoint(0.0, 8.0));
  EXPECT_DOUBLE_EQ(r, 5.0);
}

// The circumscribed radius is a property of the point set, not their order:
// permuting the three arguments must give the same radius.
TEST(CircumscribedRadius, InvariantToPointOrder)
{
  const Point a = makePoint(0.0, 0.0);
  const Point b = makePoint(6.0, 0.0);
  const Point c = makePoint(0.0, 8.0);
  const double r_abc = circumscribedRadius(a, b, c);
  EXPECT_DOUBLE_EQ(circumscribedRadius(b, c, a), r_abc);
  EXPECT_DOUBLE_EQ(circumscribedRadius(c, a, b), r_abc);
  EXPECT_DOUBLE_EQ(circumscribedRadius(a, c, b), r_abc);
}

// A non-finite coordinate (NaN/Inf from a wild pose or transform) must not
// propagate: the radius collapses to infinity -> factor 1.0 (no slowdown), never
// a non-finite factor into the commanded surge.
TEST(CircumscribedRadius, NonFiniteCoordinateIsInfinite)
{
  const double nan_c = std::numeric_limits<double>::quiet_NaN();
  const double r_nan = circumscribedRadius(
    makePoint(nan_c, 0.0), makePoint(6.0, 0.0), makePoint(0.0, 8.0));
  EXPECT_TRUE(std::isinf(r_nan));
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(r_nan, 50.0, 0.3), 1.0);

  const double inf_c = std::numeric_limits<double>::infinity();
  const double r_inf = circumscribedRadius(
    makePoint(0.0, 0.0), makePoint(inf_c, 0.0), makePoint(0.0, 8.0));
  EXPECT_TRUE(std::isinf(r_inf));
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(r_inf, 50.0, 0.3), 1.0);
}

// Fewer than three distinct points -> no circle is defined -> infinite radius.
// The near-goal case the operator flagged: as the look-ahead runs onto the goal,
// lookaheadPoint clamps the half- and full-lookahead points onto the same final
// vertex, so all three fit points coincide (or two of them do). Must be a clean
// no-op (factor 1.0), not a division-by-zero NaN into the surge.
TEST(CircumscribedRadius, CoincidentPointsAreInfinite)
{
  // All three coincident (boat parked on the goal).
  const double r_all = circumscribedRadius(
    makePoint(5.0, 5.0), makePoint(5.0, 5.0), makePoint(5.0, 5.0));
  EXPECT_TRUE(std::isinf(r_all));
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(r_all, 50.0, 0.3), 1.0);

  // Near-goal: half_lookahead == full_lookahead == goal (two points coincide).
  const double r_two = circumscribedRadius(
    makePoint(0.0, 0.0), makePoint(10.0, 10.0), makePoint(10.0, 10.0));
  EXPECT_TRUE(std::isinf(r_two));
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(r_two, 50.0, 0.3), 1.0);
}

// Disabled: min_radius <= 0 returns 1.0 for any radius (the shipped default, so
// there is no behavior change until a platform opts in).
TEST(CurvatureSpeedFactor, DisabledReturnsOne)
{
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(5.0, 0.0, 0.3), 1.0);
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(1.0, 0.0, 0.3), 1.0);
  // A negative min_radius is also treated as disabled, not a sign flip.
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(5.0, -10.0, 0.3), 1.0);
}

// A gentle bend (radius >= min_radius) leaves the speed untouched.
TEST(CurvatureSpeedFactor, GentleBendReturnsOne)
{
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(50.0, 40.0, 0.3), 1.0);   // radius > min
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(40.0, 40.0, 0.3), 1.0);   // exactly at min
  // An infinite radius (a straight line) is the ultimate gentle bend.
  EXPECT_DOUBLE_EQ(
    curvatureSpeedFactor(std::numeric_limits<double>::infinity(), 40.0, 0.3), 1.0);
}

// A tight turn (radius < min_radius) ramps linearly as radius / min_radius.
TEST(CurvatureSpeedFactor, TightTurnRampsLinearly)
{
  // R = 5, min = 10 -> 0.5 (above the 0.3 floor).
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(5.0, 10.0, 0.3), 0.5);
  // R = 30, min = 40 -> 0.75.
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(30.0, 40.0, 0.0), 0.75);
  // The right-triangle radius (R = 5) with a 10 m engagement radius -> 0.5,
  // exercising circumscribedRadius and curvatureSpeedFactor together.
  const double r = circumscribedRadius(
    makePoint(0.0, 0.0), makePoint(6.0, 0.0), makePoint(0.0, 8.0));
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(r, 10.0, 0.3), 0.5);
}

// The floor holds: a very tight turn never drops below min_factor, and the
// returned factor is always in [min_factor, 1.0].
TEST(CurvatureSpeedFactor, MinFactorFloorRespected)
{
  // R tiny relative to min -> would be far below the floor; the clamp holds it.
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(1.0, 100.0, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(curvatureSpeedFactor(0.001, 50.0, 0.25), 0.25);

  // Sweep radius across the range: result stays within [min_factor, 1.0].
  const double min_factor = 0.4;
  const double min_radius = 40.0;
  for (double radius = 0.0; radius <= 80.0; radius += 2.0) {
    const double f = curvatureSpeedFactor(radius, min_radius, min_factor);
    EXPECT_GE(f, min_factor);
    EXPECT_LE(f, 1.0);
    EXPECT_TRUE(std::isfinite(f));
  }
}

// --- End-to-end wiring over a multi-segment plan -------------------------
// These exercise the full anticipatory-curvature path the calculate() site
// walks (crabbing_path_follower.cpp:991-1006): the along-track foot, the
// half-lookahead, and the full-lookahead points are selected off a real
// multi-segment global_plan_ via lookaheadPoint(), fed to circumscribedRadius /
// curvatureSpeedFactor, and composed with the reactive turnSpeedFactor via min().

// A straight multi-segment plan curves nowhere: the three fit points are
// collinear, so the curvature factor is a clean 1.0 (no slowdown) and the
// combined factor is governed entirely by the reactive turn factor.
TEST(CurvatureWiring, StraightPlanNeverSlowsForCurvature)
{
  const auto plan = straightPlan();
  const auto r = regulate(
    plan, /*current_segment=*/0, /*progress=*/0.0, /*lookahead=*/30.0,
    /*crab_angle_deg=*/0.0, /*max_crab_deg=*/30.0, /*min_factor=*/0.3,
    /*curvature_min_radius=*/25.0);
  EXPECT_DOUBLE_EQ(r.curvature_factor, 1.0);
  EXPECT_DOUBLE_EQ(r.combined, r.turn_factor);  // reactive governs
}

// The L-shaped plan with lookahead 30 m selects foot (0,0), half (15,0), and
// full (20,10) — the full-lookahead point runs 10 m past the corner into the
// second segment. Those three points circumscribe a circle of radius exactly
// 12.5 m; with a 25 m engagement radius the curvature factor is 12.5/25 = 0.5.
// This pins the foot/half/full selection *and* the geometry in one shot.
TEST(CurvatureWiring, CorneredPlanSlowsAnticipatingTheTurn)
{
  const auto plan = lShapedPlan();
  const auto r = regulate(
    plan, /*current_segment=*/0, /*progress=*/0.0, /*lookahead=*/30.0,
    /*crab_angle_deg=*/0.0, /*max_crab_deg=*/0.0, /*min_factor=*/0.3,
    /*curvature_min_radius=*/25.0);
  EXPECT_DOUBLE_EQ(r.curvature_factor, 0.5);
  // Reactive regulator disabled (max_crab_deg = 0 -> turn_factor 1.0), so the
  // anticipatory curvature factor is what reaches the surge.
  EXPECT_DOUBLE_EQ(r.turn_factor, 1.0);
  EXPECT_DOUBLE_EQ(r.combined, 0.5);
}

// The along-track foot is picked at `progress` metres into the current segment,
// not the segment start: starting 5 m along segment 0 shifts all three fit
// points forward, but the plan still curves, so the boat still slows. Exercises
// the start_offset path through lookaheadPoint that the call site relies on.
TEST(CurvatureWiring, AlongTrackFootHonoursProgress)
{
  const auto plan = lShapedPlan();
  const auto r = regulate(
    plan, /*current_segment=*/0, /*progress=*/5.0, /*lookahead=*/30.0,
    /*crab_angle_deg=*/0.0, /*max_crab_deg=*/0.0, /*min_factor=*/0.3,
    /*curvature_min_radius=*/1000.0);
  EXPECT_LT(r.curvature_factor, 1.0);
  EXPECT_GE(r.curvature_factor, 0.3);
}

// min() composition: whichever regulator demands the greater slowdown wins.
// Reactive harder — a hard crab on a straight plan: turn_factor floors while the
// curvature factor stays 1.0, so the reactive factor governs the combined result.
TEST(CurvatureWiring, MinCompositionReactiveGoverns)
{
  const auto plan = straightPlan();
  const auto r = regulate(
    plan, /*current_segment=*/0, /*progress=*/0.0, /*lookahead=*/30.0,
    /*crab_angle_deg=*/30.0, /*max_crab_deg=*/30.0, /*min_factor=*/0.3,
    /*curvature_min_radius=*/25.0);
  EXPECT_DOUBLE_EQ(r.turn_factor, 0.3);       // |crab| == max_crab -> floor
  EXPECT_DOUBLE_EQ(r.curvature_factor, 1.0);  // straight -> no curvature slowdown
  EXPECT_DOUBLE_EQ(r.combined, 0.3);          // reactive governs
}

// Anticipatory harder — a gentle crab into the L-shaped corner: the curvature
// factor (0.5) is below the reactive factor, so the curvature factor governs.
TEST(CurvatureWiring, MinCompositionCurvatureGoverns)
{
  const auto plan = lShapedPlan();
  // max_crab 60, |crab| 30 -> turn_factor clamp(1 - 30/60, .3, 1) = 0.5... make
  // the reactive factor clearly milder than the 0.5 curvature factor: |crab| 12
  // of 60 -> 0.8.
  const auto r = regulate(
    plan, /*current_segment=*/0, /*progress=*/0.0, /*lookahead=*/30.0,
    /*crab_angle_deg=*/12.0, /*max_crab_deg=*/60.0, /*min_factor=*/0.3,
    /*curvature_min_radius=*/25.0);
  EXPECT_DOUBLE_EQ(r.turn_factor, 0.8);
  EXPECT_DOUBLE_EQ(r.curvature_factor, 0.5);
  EXPECT_DOUBLE_EQ(r.combined, 0.5);  // anticipatory governs
}

// The whole anticipatory path is gated off when the feature is disabled
// (curvature_min_radius <= 0, the shipped default) even on a sharply curved
// plan: the curvature factor is 1.0 and only the reactive factor can slow the boat.
TEST(CurvatureWiring, DisabledCurvatureRadiusIsNoOpOnCurvedPlan)
{
  const auto plan = lShapedPlan();
  const auto r = regulate(
    plan, /*current_segment=*/0, /*progress=*/0.0, /*lookahead=*/30.0,
    /*crab_angle_deg=*/0.0, /*max_crab_deg=*/0.0, /*min_factor=*/0.3,
    /*curvature_min_radius=*/0.0);
  EXPECT_DOUBLE_EQ(r.curvature_factor, 1.0);
  EXPECT_DOUBLE_EQ(r.combined, 1.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
