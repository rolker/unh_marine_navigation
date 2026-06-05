#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "marine_nav_ca_safety/ca_safety.h"

using namespace marine_nav_ca_safety;  // NOLINT(build/namespaces)

namespace
{
SafetyParams defaultParams()
{
  return SafetyParams{};  // struct defaults
}
}  // namespace

TEST(IsFinitePositive, RejectsNonPositiveAndNonFinite)
{
  EXPECT_TRUE(isFinitePositive(0.1));
  EXPECT_FALSE(isFinitePositive(0.0));
  EXPECT_FALSE(isFinitePositive(-1.0));
  EXPECT_FALSE(isFinitePositive(std::nan("")));
  EXPECT_FALSE(isFinitePositive(std::numeric_limits<double>::infinity()));
}

TEST(SlowdownLength, ScalesWithSpeedAndClamps)
{
  SafetyParams p = defaultParams();
  p.ttc_time_constant = 4.0;
  p.slowdown_min_length = 5.0;
  p.slowdown_max_length = 25.0;

  EXPECT_DOUBLE_EQ(slowdownLength(0.0, p), 5.0);      // floor at min
  EXPECT_DOUBLE_EQ(slowdownLength(-3.0, p), 5.0);     // negative treated as 0
  EXPECT_DOUBLE_EQ(slowdownLength(2.0, p), 13.0);     // 2*4 + 5
  EXPECT_DOUBLE_EQ(slowdownLength(100.0, p), 25.0);   // clamp at max
}

TEST(SlowdownLength, RobustToMisorderedRange)
{
  SafetyParams p = defaultParams();
  p.slowdown_min_length = 20.0;
  p.slowdown_max_length = 5.0;  // misordered
  // Must not invoke UB; result stays >= min.
  const double len = slowdownLength(0.0, p);
  EXPECT_GE(len, 20.0);
  EXPECT_TRUE(std::isfinite(len));
}

TEST(NearestForwardRange, FindsNearestInsideBoxAndIgnoresOutside)
{
  std::vector<Point2> pts{
    {3.0, 0.0},    // inside, nearest
    {5.0, 1.0},    // inside, farther
    {-2.0, 0.0},   // behind -> ignored
    {4.0, 10.0},   // too wide -> ignored
    {30.0, 0.0}};  // beyond max_x -> ignored
  EXPECT_DOUBLE_EQ(nearestForwardRange(pts, /*width=*/6.0, /*max_x=*/20.0), 3.0);
}

TEST(NearestForwardRange, ReturnsInfWhenEmptyOrAllOutside)
{
  EXPECT_TRUE(std::isinf(nearestForwardRange({}, 6.0, 20.0)));
  std::vector<Point2> pts{{-1.0, 0.0}, {2.0, 9.0}};
  EXPECT_TRUE(std::isinf(nearestForwardRange(pts, 6.0, 20.0)));
}

TEST(NearestForwardRange, IgnoresNonFinitePoints)
{
  std::vector<Point2> pts{
    {std::nan(""), 0.0}, {std::numeric_limits<double>::infinity(), 0.0}, {4.0, 0.0}};
  EXPECT_DOUBLE_EQ(nearestForwardRange(pts, 6.0, 20.0), 4.0);
}

TEST(Classify, StopDominatesSlowdownAndClear)
{
  EXPECT_EQ(classify(/*stop=*/3.0, /*slow=*/3.0, /*len=*/15.0), Zone::Stop);
  EXPECT_EQ(
    classify(std::numeric_limits<double>::infinity(), 10.0, 15.0), Zone::Slowdown);
  EXPECT_EQ(
    classify(
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 15.0),
    Zone::Clear);
  // Slow range beyond the dynamic length -> Clear.
  EXPECT_EQ(classify(std::numeric_limits<double>::infinity(), 20.0, 15.0), Zone::Clear);
}

TEST(SlowdownScale, RampsFromStopToSlowdown)
{
  EXPECT_DOUBLE_EQ(slowdownScale(15.0, 15.0, 5.0), 1.0);  // at outer edge
  EXPECT_DOUBLE_EQ(slowdownScale(20.0, 15.0, 5.0), 1.0);  // beyond -> clamped 1
  EXPECT_DOUBLE_EQ(slowdownScale(5.0, 15.0, 5.0), 0.0);   // at stop boundary
  EXPECT_DOUBLE_EQ(slowdownScale(10.0, 15.0, 5.0), 0.5);  // midpoint
  // Degenerate range -> hard step at stop_len.
  EXPECT_DOUBLE_EQ(slowdownScale(6.0, 5.0, 5.0), 1.0);
  EXPECT_DOUBLE_EQ(slowdownScale(4.0, 5.0, 5.0), 0.0);
}

TEST(ApplySlowdown, ReducesForwardKeepsFloorPreservesYaw)
{
  Twist2 in{2.0, 0.7};
  // scale 0.5 -> 1.0, but floor keeps it above 0.1
  Twist2 out = applySlowdown(in, 0.5, 0.1);
  EXPECT_DOUBLE_EQ(out.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.7);  // yaw untouched

  // scale 0 -> floored to speed_floor (not zero) so yaw authority survives
  out = applySlowdown(in, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.1);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.7);

  // never speeds up
  out = applySlowdown(in, 5.0, 0.1);
  EXPECT_DOUBLE_EQ(out.linear_x, 2.0);
}

TEST(ApplySlowdown, LeavesNonPositiveForwardAlone)
{
  Twist2 in{-0.5, 0.3};
  Twist2 out = applySlowdown(in, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(out.linear_x, -0.5);  // not driving forward; untouched
  EXPECT_DOUBLE_EQ(out.angular_z, 0.3);
}

TEST(ApplyStop, ReverseWhileMovingHoldZeroWhenStopped)
{
  SafetyParams p = defaultParams();
  p.reverse_speed = 0.6;
  p.cancel_yaw_during_reverse = true;
  Twist2 in{1.0, 0.4};

  // moving forward + allowed -> reverse thrust, yaw cancelled
  Twist2 out = applyStop(in, /*measured=*/0.8, /*allowed=*/true, p, /*eps=*/0.05);
  EXPECT_DOUBLE_EQ(out.linear_x, -0.6);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);

  // essentially stopped -> hold zero
  out = applyStop(in, /*measured=*/0.01, /*allowed=*/true, p, /*eps=*/0.05);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);

  // backstop hit (not allowed) -> hold zero even if moving
  out = applyStop(in, /*measured=*/0.8, /*allowed=*/false, p, /*eps=*/0.05);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
}

TEST(ApplyStop, YawPassthroughWhenNotCancelled)
{
  SafetyParams p = defaultParams();
  p.cancel_yaw_during_reverse = false;
  Twist2 in{1.0, 0.4};
  Twist2 out = applyStop(in, 0.8, true, p, 0.05);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.4);
}

TEST(ForwardBoxCorners, ProducesCenteredForwardRectangle)
{
  const auto c = forwardBoxCorners(10.0, 4.0);
  EXPECT_DOUBLE_EQ(c[0].x, 10.0);
  EXPECT_DOUBLE_EQ(c[0].y, 2.0);
  EXPECT_DOUBLE_EQ(c[2].x, 0.0);
  EXPECT_DOUBLE_EQ(c[2].y, -2.0);
}
