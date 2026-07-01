#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

using marine_nav_crabbing_path_follower::turnSpeedFactor;

// Disabled: max_crab_deg <= 0 returns 1.0 at any crab angle (the shipped
// default, so there is no behavior change until a platform opts in).
TEST(TurnSpeedFactor, DisabledReturnsOne)
{
  EXPECT_DOUBLE_EQ(turnSpeedFactor(0.0, 0.0, 0.3), 1.0);
  EXPECT_DOUBLE_EQ(turnSpeedFactor(45.0, 0.0, 0.3), 1.0);
  EXPECT_DOUBLE_EQ(turnSpeedFactor(90.0, 0.0, 0.3), 1.0);
  // A negative max_crab_deg is also treated as disabled (identity), not a sign
  // flip — the validator rejects it, but the pure function is defensive.
  EXPECT_DOUBLE_EQ(turnSpeedFactor(45.0, -10.0, 0.3), 1.0);
}

// Zero crab (a straight run, controller on-track) -> no slowdown.
TEST(TurnSpeedFactor, ZeroCrabReturnsOne)
{
  EXPECT_DOUBLE_EQ(turnSpeedFactor(0.0, 30.0, 0.3), 1.0);
  EXPECT_DOUBLE_EQ(turnSpeedFactor(0.0, 90.0, 0.5), 1.0);
}

// Crab at (or beyond) max_crab_deg -> the min_factor floor (maximum regulation).
TEST(TurnSpeedFactor, CrabAtMaxReturnsMinFactor)
{
  EXPECT_DOUBLE_EQ(turnSpeedFactor(30.0, 30.0, 0.3), 0.3);   // exactly at max
  EXPECT_DOUBLE_EQ(turnSpeedFactor(45.0, 30.0, 0.3), 0.3);   // beyond max
  EXPECT_DOUBLE_EQ(turnSpeedFactor(90.0, 30.0, 0.3), 0.3);
}

// Intermediate crab ramps linearly: 1 - |crab| / max_crab_deg.
TEST(TurnSpeedFactor, LinearRamp)
{
  // max = 40, crab = 20 -> 1 - 0.5 = 0.5.
  EXPECT_DOUBLE_EQ(turnSpeedFactor(20.0, 40.0, 0.0), 0.5);
  // max = 60, crab = 15 -> 1 - 0.25 = 0.75.
  EXPECT_DOUBLE_EQ(turnSpeedFactor(15.0, 60.0, 0.0), 0.75);
  // max = 30, crab = 6 -> 1 - 0.2 = 0.8 (above the 0.3 floor).
  EXPECT_DOUBLE_EQ(turnSpeedFactor(6.0, 30.0, 0.3), 0.8);
}

// Regulation depends on |crab|: a negative crab slows the boat identically to
// the same-magnitude positive crab.
TEST(TurnSpeedFactor, SymmetricForNegativeCrab)
{
  EXPECT_DOUBLE_EQ(turnSpeedFactor(-20.0, 40.0, 0.0), turnSpeedFactor(20.0, 40.0, 0.0));
  EXPECT_DOUBLE_EQ(turnSpeedFactor(-45.0, 30.0, 0.3), 0.3);   // beyond max, floored
  EXPECT_DOUBLE_EQ(turnSpeedFactor(-6.0, 30.0, 0.3), 0.8);
}

// A non-finite crab angle (NaN/Inf from a wild PID output) must not propagate a
// non-finite factor into the surge: it clamps to the min_factor floor (slow the
// boat rather than command a wild speed).
TEST(TurnSpeedFactor, NonFiniteCrabClampsToMinFactor)
{
  const double nan_c = std::numeric_limits<double>::quiet_NaN();
  const double inf_c = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(turnSpeedFactor(nan_c, 30.0, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(turnSpeedFactor(inf_c, 30.0, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(turnSpeedFactor(-inf_c, 30.0, 0.25), 0.25);
}

// The factor never drops below min_factor, even as crab exceeds max_crab_deg,
// and the returned factor is always in [min_factor, 1.0].
TEST(TurnSpeedFactor, MinFactorFloor)
{
  // A tiny max with a large crab would give a hugely negative 1 - frac; the
  // clamp holds it at the floor.
  EXPECT_DOUBLE_EQ(turnSpeedFactor(1000.0, 5.0, 0.3), 0.3);
  // Sweep crab across the range: result stays within [min_factor, 1.0].
  const double min_factor = 0.4;
  for (double crab = 0.0; crab <= 90.0; crab += 5.0) {
    const double f = turnSpeedFactor(crab, 45.0, min_factor);
    EXPECT_GE(f, min_factor);
    EXPECT_LE(f, 1.0);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
