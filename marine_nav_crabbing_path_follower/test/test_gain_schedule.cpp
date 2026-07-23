#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

using marine_nav_crabbing_path_follower::clampPostScheduleCrab;
using marine_nav_crabbing_path_follower::gainScheduleScale;
using marine_nav_crabbing_path_follower::kPostScheduleCrabClampDeg;

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

// A negative target_speed (a wild / backwards estimate) is floored to v_min by
// the same std::max as the sub-floor creep case — the effective divisor is
// v_min, never the negative value (which would flip the gain's sign).
TEST(GainScheduleScale, FloorsNegativeTargetSpeedAtVMin)
{
  // v_min = 0.5, target_speed = -2.0 -> divides by 0.5, not -2.0.
  EXPECT_DOUBLE_EQ(gainScheduleScale(10.0, 2.0, 0.5, -2.0), 10.0 * 2.0 / 0.5);
  // Sign is preserved (not flipped by a negative divisor).
  EXPECT_GT(gainScheduleScale(8.0, 1.8, 0.5, -1.0), 0.0);
}

// A non-finite target_speed (NaN/+Inf from a stale or wild speed estimate) must
// not propagate into the crab command: the isfinite guard falls back to v_min
// so the result stays finite for finite crab_angle/gain_ref_speed/v_min.
TEST(GainScheduleScale, NonFiniteTargetSpeedStaysFinite)
{
  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  const double r_nan = gainScheduleScale(10.0, 2.0, 0.5, nan_v);
  EXPECT_TRUE(std::isfinite(r_nan));
  EXPECT_DOUBLE_EQ(r_nan, 10.0 * 2.0 / 0.5);  // fell back to v_min

  const double inf_v = std::numeric_limits<double>::infinity();
  const double r_inf = gainScheduleScale(10.0, 2.0, 0.5, inf_v);
  EXPECT_TRUE(std::isfinite(r_inf));
  EXPECT_DOUBLE_EQ(r_inf, 10.0 * 2.0 / 0.5);  // fell back to v_min
}

// Sign preservation: a negative crab angle stays negative after scaling (the
// scale factor is strictly positive).
TEST(GainScheduleScale, PreservesSign)
{
  EXPECT_LT(gainScheduleScale(-8.0, 1.8, 0.5, 1.0), 0.0);   // up-scaled, still negative
  EXPECT_GT(gainScheduleScale(8.0, 1.8, 0.5, 3.5), 0.0);    // down-scaled, still positive
  EXPECT_LT(gainScheduleScale(-8.0, 1.8, 0.5, 3.5), 0.0);   // down-scaled negative
}

// ---------------------------------------------------------------------------
// clampPostScheduleCrab (#100): the schedule multiplies AFTER the PID's own
// ±90° clamp, so a railed PID × factor > 1 pushed the scheduled crab past
// perpendicular — the 2026-07-21 Massabesic sail-away (bag
// 2026-07-21T17-26-41, unh_echoboats_project11#381). These tests compose the
// two helpers exactly as computeVelocityCommands does:
//   clampPostScheduleCrab(gainScheduleScale(pid_out, ref, v_min, v)).
// ---------------------------------------------------------------------------

// The clamp limit itself is the invariant: strictly inside ±90° so the
// along-track component of target_heading = base_heading + crab stays positive.
TEST(PostScheduleCrabClamp, LimitIsStrictlyInsidePerpendicular)
{
  EXPECT_LT(kPostScheduleCrabClampDeg, 90.0);
  EXPECT_GT(std::cos(kPostScheduleCrabClampDeg * M_PI / 180.0), 0.0);
}

// Railed PID (±90°) × schedule factor 1.0 (v == gain_ref_speed): the schedule
// is unity, and the raw 90° rail itself clamps to 85°.
TEST(PostScheduleCrabClamp, RailedPidAtUnityFactorClampsToLimit)
{
  const double scheduled = gainScheduleScale(90.0, 1.8, 0.5, 1.8);  // factor 1.0
  EXPECT_DOUBLE_EQ(scheduled, 90.0);
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(scheduled), kPostScheduleCrabClampDeg);
}

// Railed PID × factor 1.2 — Bizzy's field tune (gain_ref_speed=1.8, v=1.5):
// the 07-21 configuration. 90° × 1.2 = 108°, past perpendicular; must clamp.
TEST(PostScheduleCrabClamp, RailedPidAtBizzyTuneClampsPastPerpendicular)
{
  const double scheduled = gainScheduleScale(90.0, 1.8, 0.5, 1.5);  // factor 1.2
  EXPECT_DOUBLE_EQ(scheduled, 108.0);
  const double clamped = clampPostScheduleCrab(scheduled);
  EXPECT_DOUBLE_EQ(clamped, kPostScheduleCrabClampDeg);
  // The sail-away condition is gone: along-track component positive.
  EXPECT_GT(std::cos(clamped * M_PI / 180.0), 0.0);
}

// Railed PID × factor 3.6 — the low-speed edge (v = gain_v_min = 0.5 with
// gain_ref_speed = 1.8): 90° × 3.6 = 324° before wrapping; must clamp.
TEST(PostScheduleCrabClamp, RailedPidAtLowSpeedFactorClampsToLimit)
{
  const double scheduled = gainScheduleScale(90.0, 1.8, 0.5, 0.5);  // factor 3.6
  EXPECT_DOUBLE_EQ(scheduled, 324.0);
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(scheduled), kPostScheduleCrabClampDeg);
}

// Sign symmetry: negative railed inputs clamp to the negative limit.
TEST(PostScheduleCrabClamp, NegativeRailClampsSymmetrically)
{
  EXPECT_DOUBLE_EQ(
    clampPostScheduleCrab(gainScheduleScale(-90.0, 1.8, 0.5, 1.5)),
    -kPostScheduleCrabClampDeg);
  EXPECT_DOUBLE_EQ(
    clampPostScheduleCrab(gainScheduleScale(-90.0, 1.8, 0.5, 0.5)),
    -kPostScheduleCrabClampDeg);
}

// In-range angles pass through untouched — the clamp only trims the
// past-perpendicular excursions, never reshapes normal steering.
TEST(PostScheduleCrabClamp, InRangeAnglesPassThrough)
{
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(0.0), 0.0);
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(30.0), 30.0);
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(-84.9), -84.9);
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(kPostScheduleCrabClampDeg), kPostScheduleCrabClampDeg);
}

// Surge bound is UNCHANGED by the clamp (plan-review must-fix 2): the
// downstream cos_crab = max(cos(crab), 0.5) floor still governs at the rail
// (cos(85°) ≈ 0.087 < 0.5), so linear.x ≤ 2 × target_speed before AND after —
// the clamp fixes the heading sign, not the surge.
TEST(PostScheduleCrabClamp, SurgeFloorStillGovernsAtTheRail)
{
  const double clamped = clampPostScheduleCrab(108.0);
  const double cos_crab = std::max(std::cos(clamped * M_PI / 180.0), 0.5);
  EXPECT_DOUBLE_EQ(cos_crab, 0.5);            // floor active -> 2x multiplier
  EXPECT_DOUBLE_EQ(1.0 / cos_crab, 2.0);      // surge ceiling: 2 x target_speed
}

// A non-finite scheduled angle (wild upstream value) returns 0.0 — follow the
// base course rather than propagate NaN into target_heading (same fail-safe
// idiom as turnSpeedFactor's non-finite branch).
TEST(PostScheduleCrabClamp, NonFiniteReturnsZero)
{
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(std::numeric_limits<double>::quiet_NaN()), 0.0);
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(std::numeric_limits<double>::infinity()), 0.0);
  EXPECT_DOUBLE_EQ(clampPostScheduleCrab(-std::numeric_limits<double>::infinity()), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
