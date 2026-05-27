// Unit tests for PredictStoppingPose::projectStoppingPose — the pure momentum
// projection, exercised without a tf/blackboard fixture. Guards the body->world
// rotation (the easy-to-miss gotcha), the crabbing term, the v^2/(2|a|) stop
// distance, and the no-projection guards.

#include <cmath>

#include <gtest/gtest.h>

#include <tf2/LinearMath/Quaternion.h>  // NOLINT(build/include_order)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "marine_nav_behavior_tree/plugins/action/predict_stopping_pose.h"

using marine_nav_behavior_tree::PredictStoppingPose;

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

geometry_msgs::msg::Twist bodyTwist(double vx, double vy)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = vx;
  t.linear.y = vy;
  return t;
}

constexpr double kDecel = -0.45;
constexpr double kTol = 1e-3;

}  // namespace

// Heading 0: a 1.5 m/s forward entry projects straight ahead in +x.
// distance = v^2 / (2|a|) = 1.5^2 / (2*0.45) = 2.5 m.
TEST(PredictStoppingPose, ForwardHeadingZero)
{
  const auto stop = PredictStoppingPose::projectStoppingPose(
    makePose(0.0, 0.0, 0.0), bodyTwist(1.5, 0.0), kDecel);
  EXPECT_NEAR(stop.pose.position.x, 2.5, kTol);
  EXPECT_NEAR(stop.pose.position.y, 0.0, kTol);
}

// Heading 90 deg: the same body-forward velocity must rotate into +y, not +x.
// This is the body->world rotation that the legacy port was careful about.
TEST(PredictStoppingPose, HeadingNinetyRotatesIntoWorldY)
{
  const auto stop = PredictStoppingPose::projectStoppingPose(
    makePose(0.0, 0.0, M_PI_2), bodyTwist(1.5, 0.0), kDecel);
  EXPECT_NEAR(stop.pose.position.x, 0.0, kTol);
  EXPECT_NEAR(stop.pose.position.y, 2.5, kTol);
}

// Pure crabbing (body +y, no forward) must still project — the full 2D
// velocity is used, not just forward speed. distance = 1.0^2/(2*0.45)=1.111 m.
TEST(PredictStoppingPose, CrabbingContributes)
{
  const auto stop = PredictStoppingPose::projectStoppingPose(
    makePose(0.0, 0.0, 0.0), bodyTwist(0.0, 1.0), kDecel);
  EXPECT_NEAR(stop.pose.position.x, 0.0, kTol);
  EXPECT_NEAR(stop.pose.position.y, 1.0 * 1.0 / (2.0 * 0.45), kTol);
}

// Offset start pose: projection is added to the current position, frame and z
// are preserved.
TEST(PredictStoppingPose, AddsToCurrentPositionAndPreservesFrame)
{
  auto start = makePose(10.0, -5.0, 0.0);
  start.pose.position.z = 3.0;
  const auto stop = PredictStoppingPose::projectStoppingPose(
    start, bodyTwist(1.5, 0.0), kDecel);
  EXPECT_NEAR(stop.pose.position.x, 12.5, kTol);
  EXPECT_NEAR(stop.pose.position.y, -5.0, kTol);
  EXPECT_NEAR(stop.pose.position.z, 3.0, kTol);
  EXPECT_EQ(stop.header.frame_id, "map");
}

// A stationary boat holds its current pose (no projection).
TEST(PredictStoppingPose, ZeroVelocityHoldsCurrentPose)
{
  const auto start = makePose(4.0, 2.0, 1.0);
  const auto stop = PredictStoppingPose::projectStoppingPose(
    start, bodyTwist(0.0, 0.0), kDecel);
  EXPECT_NEAR(stop.pose.position.x, 4.0, kTol);
  EXPECT_NEAR(stop.pose.position.y, 2.0, kTol);
}

// A non-negative "deceleration" is a misconfiguration; hold current pose rather
// than projecting to infinity or the wrong direction.
TEST(PredictStoppingPose, NonNegativeDecelHoldsCurrentPose)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto stop = PredictStoppingPose::projectStoppingPose(
    start, bodyTwist(2.0, 0.0), 0.0);
  EXPECT_NEAR(stop.pose.position.x, 0.0, kTol);
  EXPECT_NEAR(stop.pose.position.y, 0.0, kTol);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
