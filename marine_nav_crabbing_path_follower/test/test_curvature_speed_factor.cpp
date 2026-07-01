#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

using geometry_msgs::msg::Point;
using marine_nav_crabbing_path_follower::circumscribedRadius;
using marine_nav_crabbing_path_follower::curvatureSpeedFactor;

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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
