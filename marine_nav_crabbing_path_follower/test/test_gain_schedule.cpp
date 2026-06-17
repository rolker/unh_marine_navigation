#include <cmath>

#include <gtest/gtest.h>

#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

using marine_nav_crabbing_path_follower::gainScheduleScale;

// Disabled: gain_ref_speed <= 0 returns the crab angle unchanged (the default,
// so there is no behavior change until a platform opts in).
TEST(GainScheduleScale, DisabledReturnsInputUnchanged)
{
  // gain_ref_speed == 0 (the shipped default) is identity at any speed.
  EXPECT_DOUBLE_EQ(gainScheduleScale(12.0, 0.0, 0.5, 1.0), 12.0);
  EXPECT_DOUBLE_EQ(gainScheduleScale(12.0, 0.0, 0.5, 3.5), 12.0);
  EXPECT_DOUBLE_EQ(gainScheduleScale(-7.5, 0.0, 0.5, 2.0), -7.5);
  // A negative gain_ref_speed is also treated as disabled (identity), not a
  // sign flip — the validator rejects it, but the pure function is defensive.
  EXPECT_DOUBLE_EQ(gainScheduleScale(12.0, -1.0, 0.5, 1.0), 12.0);
}

// Enabled, at the reference speed: v == gain_ref_speed -> unity (unchanged).
TEST(GainScheduleScale, UnityAtReferenceSpeed)
{
  EXPECT_DOUBLE_EQ(gainScheduleScale(10.0, 1.8, 0.5, 1.8), 10.0);
  EXPECT_DOUBLE_EQ(gainScheduleScale(-4.0, 2.0, 0.5, 2.0), -4.0);
}

// Enabled, below the reference speed: scales the gain UP (ref/v > 1).
TEST(GainScheduleScale, ScalesUpBelowReferenceSpeed)
{
  // ref = 2.0, v = 1.0 -> factor 2.0.
  EXPECT_DOUBLE_EQ(gainScheduleScale(6.0, 2.0, 0.5, 1.0), 12.0);
  // ref = 1.8, v = 1.2 -> factor 1.5.
  EXPECT_DOUBLE_EQ(gainScheduleScale(4.0, 1.8, 0.5, 1.2), 4.0 * 1.8 / 1.2);
}

// Enabled, above the reference speed: scales the gain DOWN (ref/v < 1) — the
// point of the fix (less aggressive cross-track gain as speed rises).
TEST(GainScheduleScale, ScalesDownAboveReferenceSpeed)
{
  // ref = 2.0, v = 4.0 -> factor 0.5.
  EXPECT_DOUBLE_EQ(gainScheduleScale(10.0, 2.0, 0.5, 4.0), 5.0);
  // ref = 1.8, v = 3.5 -> factor 1.8/3.5.
  EXPECT_DOUBLE_EQ(gainScheduleScale(7.0, 1.8, 0.5, 3.5), 7.0 * 1.8 / 3.5);
}

// gain_v_min floor: a target_speed below v_min divides by v_min (not the lower
// target_speed), so the gain stays bounded at creep / station-keep.
TEST(GainScheduleScale, FloorsDivisorAtVMin)
{
  // v_min = 0.5, target_speed = 0.1 (< v_min) -> divides by 0.5, not 0.1.
  EXPECT_DOUBLE_EQ(gainScheduleScale(10.0, 2.0, 0.5, 0.1), 10.0 * 2.0 / 0.5);
  // Exactly at the floor: divides by v_min.
  EXPECT_DOUBLE_EQ(gainScheduleScale(10.0, 2.0, 0.5, 0.5), 10.0 * 2.0 / 0.5);
}

// target_speed == 0 (station-keep / start of motion) stays finite: the floor
// prevents the divide-by-zero blow-up that would command NaN/Inf on the boat.
TEST(GainScheduleScale, ZeroTargetSpeedStaysFinite)
{
  const double r = gainScheduleScale(10.0, 2.0, 0.5, 0.0);
  EXPECT_TRUE(std::isfinite(r));
  EXPECT_DOUBLE_EQ(r, 10.0 * 2.0 / 0.5);  // divided by v_min, not 0
}

// Sign preservation: a negative crab angle stays negative after scaling (the
// scale factor is strictly positive).
TEST(GainScheduleScale, PreservesSign)
{
  EXPECT_LT(gainScheduleScale(-8.0, 1.8, 0.5, 1.0), 0.0);   // up-scaled, still negative
  EXPECT_GT(gainScheduleScale(8.0, 1.8, 0.5, 3.5), 0.0);    // down-scaled, still positive
  EXPECT_LT(gainScheduleScale(-8.0, 1.8, 0.5, 3.5), 0.0);   // down-scaled negative
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
