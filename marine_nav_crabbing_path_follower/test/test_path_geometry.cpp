#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

using marine_nav_crabbing_path_follower::lookaheadPoint;

namespace
{
std::vector<geometry_msgs::msg::PoseStamped> makePath(
  const std::vector<std::pair<double, double>> & pts)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  for (const auto & p : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = p.first;
    ps.pose.position.y = p.second;
    path.push_back(ps);
  }
  return path;
}
}  // namespace

// Walk L forward along a straight segment.
TEST(LookaheadPoint, WalksForwardAlongStraightLine)
{
  auto path = makePath({{0, 0}, {20, 0}});
  auto p = lookaheadPoint(path, 0, 5.0, 10.0);  // 5 m in, +10 m -> x = 15
  EXPECT_NEAR(p.x, 15.0, 1e-6);
  EXPECT_NEAR(p.y, 0.0, 1e-6);
}

// The worked example: the look-ahead point reaches *around* the bend, which is
// exactly the anticipation we want (boat still on the straight, point past the
// corner).
TEST(LookaheadPoint, AnticipatesAroundABend)
{
  // A(0,0) -> B(20,0) -> C(30,10): straight east, then a left bend at B.
  auto path = makePath({{0, 0}, {20, 0}, {30, 10}});
  // Projected at x = 16 on segment 0 (offset 16), look 10 m: consume 4 m to B,
  // then 6 m up B->C (unit (sqrt(0.5), sqrt(0.5))).
  auto p = lookaheadPoint(path, 0, 16.0, 10.0);
  EXPECT_NEAR(p.x, 20.0 + 6.0 * std::sqrt(0.5), 1e-3);
  EXPECT_NEAR(p.y, 6.0 * std::sqrt(0.5), 1e-3);
  EXPECT_GT(p.y, 0.0);  // past the corner == anticipating the turn
}

// Past the end of the path, clamp to the final point (the goal) so the boat
// converges onto it rather than over-running.
TEST(LookaheadPoint, ClampsToGoalPastEnd)
{
  auto path = makePath({{0, 0}, {20, 0}});
  auto p = lookaheadPoint(path, 0, 5.0, 1000.0);
  EXPECT_NEAR(p.x, 20.0, 1e-6);
  EXPECT_NEAR(p.y, 0.0, 1e-6);
}

// Honour the starting segment + offset.
TEST(LookaheadPoint, StartsFromGivenSegmentAndOffset)
{
  auto path = makePath({{0, 0}, {10, 0}, {20, 0}});
  auto p = lookaheadPoint(path, 1, 2.0, 3.0);  // seg 1 (10..20), 2 in, +3 -> x = 15
  EXPECT_NEAR(p.x, 15.0, 1e-6);
}

// Degenerate paths must not read out of bounds.
TEST(LookaheadPoint, HandlesDegeneratePaths)
{
  geometry_msgs::msg::Point z;
  EXPECT_NO_THROW(z = lookaheadPoint({}, 0, 0.0, 5.0));
  auto single = makePath({{3, 4}});
  auto p = lookaheadPoint(single, 0, 0.0, 5.0);
  EXPECT_NEAR(p.x, 3.0, 1e-6);
  EXPECT_NEAR(p.y, 4.0, 1e-6);
}

// A start segment past the end returns the goal rather than reading past the end.
TEST(LookaheadPoint, OutOfRangeStartSegmentReturnsGoal)
{
  auto path = makePath({{0, 0}, {20, 0}});
  auto p = lookaheadPoint(path, 5, 0.0, 3.0);
  EXPECT_NEAR(p.x, 20.0, 1e-6);
}

namespace
{
geometry_msgs::msg::Point pt(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}
}  // namespace

using marine_nav_crabbing_path_follower::alongTrackProjection;

// Along-track projection: sign and magnitude relative to the segment start.
TEST(AlongTrackProjection, PositiveAheadNegativeBehind)
{
  auto a = pt(0, 0);
  auto b = pt(10, 0);          // segment points +x
  EXPECT_NEAR(alongTrackProjection(a, b, pt(4, 3)), 4.0, 1e-9);   // ahead (lateral offset ignored)
  EXPECT_NEAR(alongTrackProjection(a, b, pt(0, 5)), 0.0, 1e-9);   // at the start
  EXPECT_LT(alongTrackProjection(a, b, pt(-2, 1)), 0.0);          // behind the start -> negative
}

// The backward-correction trigger: a boat behind the current segment start.
TEST(AlongTrackProjection, NegativeIsTheBackwardStepTrigger)
{
  auto a = pt(5, 5);
  auto b = pt(5, 15);          // segment points +y
  EXPECT_LT(alongTrackProjection(a, b, pt(7, 2)), 0.0);           // below a -> behind
  EXPECT_GT(alongTrackProjection(a, b, pt(7, 9)), 0.0);           // above a -> ahead
}

// Degenerate (zero-length) segment must not divide by zero.
TEST(AlongTrackProjection, ZeroLengthSegmentReturnsZero)
{
  auto a = pt(3, 3);
  EXPECT_EQ(alongTrackProjection(a, a, pt(9, 9)), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
