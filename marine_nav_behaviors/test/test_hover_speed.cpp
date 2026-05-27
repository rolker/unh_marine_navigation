// Unit tests for computeHoverSpeed (the field-tuned v4 speed-magnitude logic)
// and its composition with chooseApproachHeading into a signed linear.x — the
// combined forward/reverse path that Hover::onCycleUpdate runs. The headline
// case is reverse approach inside the deadband: the negative back-off term times
// drive_sign = -1 must still push the boat *away* from the hold point.

#include <gtest/gtest.h>

#include <cmath>

#include "marine_nav_behaviors/hover_heading.h"
#include "marine_nav_behaviors/hover_speed.h"

using marine_nav_behaviors::chooseApproachHeading;
using marine_nav_behaviors::computeHoverSpeed;

namespace
{
// Mirror the seafloor/ben behavior_server.hover params.
constexpr double kMinR = 2.5;
constexpr double kMaxR = 10.0;
constexpr double kMinSpeed = 0.0;
constexpr double kMaxSpeed = 1.0;
constexpr double kTol = 1e-6;

// Reproduce the exact production composition in hover.cpp:onCycleUpdate:
// choose the approach heading from the forward heading error, derive the
// steering proportion from the chosen angle, compute the speed magnitude, and
// apply the drive sign.
double commandedLinearX(double forward_error, bool point_at_target, double range)
{
  const auto approach = chooseApproachHeading(forward_error, point_at_target);
  const double steering_proportion = std::abs(approach.steering_angle) / M_PI;
  const double speed = computeHoverSpeed(
    range, steering_proportion, kMinR, kMaxR, kMinSpeed, kMaxSpeed);
  return approach.drive_sign * speed;
}
}  // namespace

// --- computeHoverSpeed: range bands (aligned, steering_proportion = 0) ---

TEST(ComputeHoverSpeed, AtOrBeyondMaxRadiusIsMaxSpeed)
{
  EXPECT_NEAR(computeHoverSpeed(kMaxR, 0.0, kMinR, kMaxR, kMinSpeed, kMaxSpeed), kMaxSpeed, kTol);
  EXPECT_NEAR(computeHoverSpeed(50.0, 0.0, kMinR, kMaxR, kMinSpeed, kMaxSpeed), kMaxSpeed, kTol);
}

TEST(ComputeHoverSpeed, RampsLinearlyBetweenRadii)
{
  // Halfway between min and max -> half of maximum_speed (aligned, no floor
  // since the floor only triggers when steering_proportion > 0.1).
  const double mid = (kMinR + kMaxR) / 2.0;
  EXPECT_NEAR(
    computeHoverSpeed(mid, 0.0, kMinR, kMaxR, kMinSpeed, kMaxSpeed), 0.5 * kMaxSpeed, kTol);
}

TEST(ComputeHoverSpeed, InnerDeadbandBacksOffNegative)
{
  // range < minimum_radius/2 -> small negative (reverse) magnitude, <= 10%.
  const double r = kMinR / 4.0;  // 0.625 m
  const double s = computeHoverSpeed(r, 0.0, kMinR, kMaxR, kMinSpeed, kMaxSpeed);
  EXPECT_LT(s, 0.0);
  EXPECT_GE(s, -0.10 * kMaxSpeed - kTol);
  // expected = -0.1 * (1 - r/min) * max
  EXPECT_NEAR(s, -0.1 * (1.0 - r / kMinR) * kMaxSpeed, kTol);
}

TEST(ComputeHoverSpeed, OuterDeadbandHoldsZero)
{
  // minimum_radius/2 <= range <= minimum_radius, aligned -> zero.
  const double r = 0.75 * kMinR;  // between min/2 and min
  EXPECT_NEAR(computeHoverSpeed(r, 0.0, kMinR, kMaxR, kMinSpeed, kMaxSpeed), 0.0, kTol);
}

// --- heading taper + turn floor interaction ---

TEST(ComputeHoverSpeed, HeadingTaperZerosForwardSpeedInsideDeadband)
{
  // Inside minimum_radius the turn floor does not apply (it requires
  // range >= minimum_radius), so a large heading error tapers speed to zero.
  const double r = 0.75 * kMinR;
  const double big_error_proportion = 0.5;  // > 0.25 -> taper factor clamps to 0
  EXPECT_NEAR(
    computeHoverSpeed(r, big_error_proportion, kMinR, kMaxR, kMinSpeed, kMaxSpeed), 0.0, kTol);
}

TEST(ComputeHoverSpeed, TurnFloorAppliesOutsideDeadbandDuringRotation)
{
  // With a heading error the taper would push speed toward zero, but outside
  // minimum_radius the range-aware floor holds it up. At range == minimum_radius
  // range_factor = 0 -> floor = 0.2 * maximum_speed.
  const double s = computeHoverSpeed(kMinR, 0.5, kMinR, kMaxR, kMinSpeed, kMaxSpeed);
  EXPECT_NEAR(s, 0.2 * kMaxSpeed, kTol);
}

// --- composition with chooseApproachHeading: the combined reverse path ---

TEST(HoverCommand, ForwardApproachFarDrivesForwardAtMax)
{
  // Aligned, far, point_at_target -> full forward speed.
  EXPECT_NEAR(commandedLinearX(0.0, true, 50.0), kMaxSpeed, kTol);
}

TEST(HoverCommand, ReverseApproachFarDrivesBackwardAtMax)
{
  // Target directly behind, anchor off -> reverse selected with zero residual
  // heading error (no taper); far -> full speed in reverse (negative linear.x).
  const double lin = commandedLinearX(M_PI, false, 50.0);
  EXPECT_LT(lin, 0.0);
  EXPECT_NEAR(lin, -kMaxSpeed, kTol);
}

TEST(HoverCommand, ReverseApproachWithResidualErrorTapersSpeed)
{
  // A target 175 deg ahead reverses with a ~5 deg residual error, so the
  // heading taper trims the backward speed below max (still negative). Guards
  // that the taper is computed from the *chosen* (reversed) heading error.
  const double e = 175.0 * M_PI / 180.0;
  const double lin = commandedLinearX(e, false, 50.0);
  EXPECT_LT(lin, 0.0);
  EXPECT_GT(lin, -kMaxSpeed);  // tapered, so strictly slower than full reverse
}

TEST(HoverCommand, ReverseApproachInsideDeadbandStillPushesAwayFromPoint)
{
  // The reviewers' headline case. Target behind, reverse selected (drive_sign
  // = -1), boat sitting inside minimum_radius/2 where the magnitude is the small
  // negative back-off term. drive_sign * negative = positive: bow-forward, i.e.
  // away from the stern-facing target — the same "ease off the point" intent the
  // forward mode expresses with a small reverse. Sign must be > 0.
  const double e = M_PI;  // directly behind -> reverse, steering ~ 0
  const double r = kMinR / 4.0;
  const double lin = commandedLinearX(e, false, r);
  EXPECT_GT(lin, 0.0);
}

TEST(HoverCommand, ForwardApproachInsideDeadbandBacksOff)
{
  // Same geometry, anchor on (always face): drive_sign = +1, so the back-off
  // term stays negative (small reverse). Opposite sign to the reverse case,
  // same physical result: ease away from the hold point.
  const double r = kMinR / 4.0;
  const double lin = commandedLinearX(0.0, true, r);
  EXPECT_LT(lin, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
