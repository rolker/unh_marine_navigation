// Unit tests for chooseApproachHeading — the hover point_at_target heading
// choice (virtual-anchor vs. ArduPilot LOIT_TYPE=0 forward/reverse).

#include <gtest/gtest.h>

#include <cmath>

#include "marine_nav_behaviors/hover_heading.h"

using marine_nav_behaviors::chooseApproachHeading;

namespace
{
constexpr double kTol = 1e-9;
}

// point_at_target == true always faces the target (forward), regardless of how
// large the heading error is.
TEST(ChooseApproachHeading, PointAtTargetAlwaysForward)
{
  for (double e : {-3.0, -1.0, 0.0, 0.5, 3.0}) {
    const auto r = chooseApproachHeading(e, true);
    EXPECT_NEAR(r.steering_angle, e, kTol);
    EXPECT_NEAR(r.drive_sign, 1.0, kTol);
  }
}

// Small forward error with the anchor off: forward is the smaller turn, so keep
// facing forward.
TEST(ChooseApproachHeading, SmallErrorPrefersForward)
{
  const auto r = chooseApproachHeading(0.3, false);
  EXPECT_NEAR(r.steering_angle, 0.3, kTol);
  EXPECT_NEAR(r.drive_sign, 1.0, kTol);
}

// Target nearly behind (forward error ~170 deg): reversing needs far less
// rotation, so drive in reverse and steer to the reversed heading.
TEST(ChooseApproachHeading, LargeErrorPrefersReverse)
{
  const double e = 170.0 * M_PI / 180.0;
  const auto r = chooseApproachHeading(e, false);
  EXPECT_NEAR(r.drive_sign, -1.0, kTol);
  // reversed heading error = e - pi (small, ~ -10 deg)
  EXPECT_NEAR(r.steering_angle, e - M_PI, kTol);
  EXPECT_LT(std::abs(r.steering_angle), std::abs(e));
}

// A target directly behind (180 deg) reverses straight back: reversed error 0.
TEST(ChooseApproachHeading, DirectlyBehindReversesStraight)
{
  const auto r = chooseApproachHeading(M_PI, false);
  EXPECT_NEAR(r.drive_sign, -1.0, kTol);
  EXPECT_NEAR(r.steering_angle, 0.0, kTol);
}

// Exactly 90 deg is a tie between forward and reverse rotation; favor forward.
TEST(ChooseApproachHeading, NinetyDegTieFavorsForward)
{
  const auto r = chooseApproachHeading(M_PI_2, false);
  EXPECT_NEAR(r.drive_sign, 1.0, kTol);
  EXPECT_NEAR(r.steering_angle, M_PI_2, kTol);
}

// Negative large error (target behind on the other side) also reverses, and the
// reversed steering error keeps the correct sign.
TEST(ChooseApproachHeading, NegativeLargeErrorReverses)
{
  const double e = -170.0 * M_PI / 180.0;
  const auto r = chooseApproachHeading(e, false);
  EXPECT_NEAR(r.drive_sign, -1.0, kTol);
  EXPECT_NEAR(r.steering_angle, e + M_PI, kTol);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
