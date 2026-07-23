// Tests for mapCursorToNewPath (#99): geometry-preserving segment-cursor
// mapping across same-goal setPlan re-issues. The regression scenario is the
// 2026-07-21 Massabesic sail-away (unh_echoboats_project11#381): under the
// avoidance decorator the follower alternates between a sparse per-waypoint
// nominal path and dense 2 m station-resampled reshapes of the same goal; the
// old index-preserving carry-over capped a dense index onto the sparse path's
// FINAL leg and the boat left an 8-waypoint return route at leg 1.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "marine_nav_crabbing_path_follower/path_geometry.hpp"

namespace mncpf = marine_nav_crabbing_path_follower;

using geometry_msgs::msg::PoseStamped;

namespace
{

std::vector<PoseStamped> makePath(const std::vector<std::pair<double, double>> & pts)
{
  std::vector<PoseStamped> poses;
  poses.reserve(pts.size());
  for (const auto & pt : pts) {
    PoseStamped p;
    p.pose.position.x = pt.first;
    p.pose.position.y = pt.second;
    poses.push_back(p);
  }
  return poses;
}

// Resample a sparse vertex path at ~`spacing` metres, keeping every vertex —
// the shape of the avoidance decorator's station-resampled output.
std::vector<PoseStamped> densify(const std::vector<PoseStamped> & sparse, double spacing)
{
  std::vector<PoseStamped> out;
  for (size_t i = 0; i + 1 < sparse.size(); ++i) {
    const auto & a = sparse[i].pose.position;
    const auto & b = sparse[i + 1].pose.position;
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(len / spacing));
    for (int k = 0; k < n; ++k) {
      const double f = static_cast<double>(k) / n;
      PoseStamped p;
      p.pose.position.x = a.x + f * (b.x - a.x);
      p.pose.position.y = a.y + f * (b.y - a.y);
      out.push_back(p);
    }
  }
  out.push_back(sparse.back());
  return out;
}

// Which sparse leg does a point lie on (nearest leg by point-to-segment
// distance)? Used to express dense-path expectations leg-wise.
int nearestLeg(const std::vector<PoseStamped> & sparse, double x, double y)
{
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < sparse.size(); ++i) {
    const auto & a = sparse[i].pose.position;
    const auto & b = sparse[i + 1].pose.position;
    const double sx = b.x - a.x;
    const double sy = b.y - a.y;
    const double len2 = sx * sx + sy * sy;
    double t = 0.0;
    if (len2 > 1e-18) {
      t = std::clamp(((x - a.x) * sx + (y - a.y) * sy) / len2, 0.0, 1.0);
    }
    const double d = std::hypot(x - (a.x + t * sx), y - (a.y + t * sy));
    if (d < best_d) {
      best_d = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

double pointToSegmentDistance(
  const std::vector<PoseStamped> & poses, int seg, double x, double y)
{
  const auto & a = poses[seg].pose.position;
  const auto & b = poses[seg + 1].pose.position;
  const double sx = b.x - a.x;
  const double sy = b.y - a.y;
  const double len2 = sx * sx + sy * sy;
  double t = 0.0;
  if (len2 > 1e-18) {
    t = std::clamp(((x - a.x) * sx + (y - a.y) * sy) / len2, 0.0, 1.0);
  }
  return std::hypot(x - (a.x + t * sx), y - (a.y + t * sy));
}

// The 2026-07-21 trackline0000 geometry (map frame, metres, rounded) — an
// 8-waypoint return route whose legs 1-6 dip ~500 m south of the survey band.
const std::vector<std::pair<double, double>> kZigzag = {
  {-787.4, -87.7}, {-732.0, -161.7}, {-696.1, -344.5}, {-650.0, -409.1},
  {-591.3, -589.4}, {-442.3, -546.7}, {-434.5, -383.4}, {-96.1, -112.0},
};

}  // namespace

// The field regression: cursor on leg 1 of the dense variant must map back to
// leg 0/1 territory of the sparse variant — never the final leg.
TEST(MapCursorToNewPath, DenseToSparseZigzagStaysOnCurrentLeg)
{
  const auto sparse = makePath(kZigzag);
  const auto dense = densify(sparse, 2.0);

  // Boat walked ~60-90 m of leg 0 (wp0->wp1 is ~92 m): dense cursor ~index 40.
  const int dense_cursor = 40;
  const auto & anchor = dense[dense_cursor].pose.position;
  ASSERT_EQ(nearestLeg(sparse, anchor.x, anchor.y), 0);

  bool fallback = true;
  const int mapped = mncpf::mapCursorToNewPath(dense, dense_cursor, sparse, &fallback);
  EXPECT_EQ(mapped, 0);
  EXPECT_FALSE(fallback);

  // The old behaviour capped to the final leg — assert we never do.
  EXPECT_NE(mapped, static_cast<int>(sparse.size()) - 2);
}

TEST(MapCursorToNewPath, SparseToDenseRoundTrip)
{
  const auto sparse = makePath(kZigzag);
  const auto dense = densify(sparse, 2.0);

  const int on_dense = mncpf::mapCursorToNewPath(sparse, 0, dense);
  const auto & p = dense[on_dense].pose.position;
  EXPECT_EQ(nearestLeg(sparse, p.x, p.y), 0);

  const int back = mncpf::mapCursorToNewPath(dense, on_dense, sparse);
  EXPECT_EQ(back, 0);
}

// A corridor-style lateral reshape (<= 6 m offsets) of the same goal must map
// to the same leg. Two flavours:
//  - mid-leg cursor (the realistic dense->dense case): exact same leg;
//  - vertex-anchored cursor (sparse old path — anchor sits ON the leg
//    boundary): landing one station before the vertex, i.e. the tail of the
//    PREVIOUS leg, is correct-by-design (the compute-side forward scan
//    re-advances within the cycle); anything further back, further forward,
//    or a distant-leg capture is a failure.
TEST(MapCursorToNewPath, LateralOffsetReshapeMapsSameLeg)
{
  const auto sparse = makePath(kZigzag);
  const auto dense = densify(sparse, 2.0);
  auto shifted_pts = kZigzag;
  for (size_t i = 1; i + 1 < shifted_pts.size(); ++i) {
    shifted_pts[i].first += 4.0;
    shifted_pts[i].second += 4.0;
  }
  const auto sparse_shifted = makePath(shifted_pts);
  const auto dense_shifted = densify(sparse_shifted, 2.0);

  // Mid-leg cursors: pick the dense station nearest each leg's midpoint and
  // map dense (unshifted) -> dense (shifted). Must stay on the same leg.
  for (int leg = 0; leg < static_cast<int>(kZigzag.size()) - 1; ++leg) {
    const double mx = 0.5 * (kZigzag[leg].first + kZigzag[leg + 1].first);
    const double my = 0.5 * (kZigzag[leg].second + kZigzag[leg + 1].second);
    int mid_cursor = 0;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(dense.size()); ++i) {
      const auto & q = dense[i].pose.position;
      const double d = std::hypot(q.x - mx, q.y - my);
      if (d < best) {
        best = d;
        mid_cursor = i;
      }
    }
    const int on_shifted = mncpf::mapCursorToNewPath(dense, mid_cursor, dense_shifted);
    const auto & p = dense_shifted[on_shifted].pose.position;
    EXPECT_EQ(nearestLeg(sparse_shifted, p.x, p.y), leg) << "mid-leg " << leg;
  }

  // Vertex-anchored cursors (sparse old path): same leg or the immediately
  // preceding one, and geometrically at the anchor (offset + one station).
  for (int leg = 0; leg < static_cast<int>(kZigzag.size()) - 1; ++leg) {
    const int on_dense = mncpf::mapCursorToNewPath(sparse, leg, dense_shifted);
    const auto & p = dense_shifted[on_dense].pose.position;
    const int mapped_leg = nearestLeg(sparse_shifted, p.x, p.y);
    EXPECT_TRUE(mapped_leg == leg || mapped_leg == leg - 1)
      << "leg " << leg << " mapped to " << mapped_leg;
    const auto & anchor = sparse[leg].pose.position;
    EXPECT_LT(std::hypot(p.x - anchor.x, p.y - anchor.y), 4.0 * std::sqrt(2.0) + 2.0 + 0.5)
      << "leg " << leg;
  }
}

// Adversarial parallel-leg case: boustrophedon legs 8 m apart; the old path is
// the current leg reshaped 6 m toward the neighbour, the new path is the
// nominal. The anchor is then strictly nearer the WRONG (adjacent) leg in
// space (2 m vs 6 m) — only the arc-length window keeps the mapping on the
// current leg.
TEST(MapCursorToNewPath, AdjacentParallelLegNotCaptured)
{
  const std::vector<std::pair<double, double>> nominal_pts = {
    {0.0, 0.0}, {100.0, 0.0}, {100.0, 8.0}, {0.0, 8.0},
  };
  auto reshaped_pts = nominal_pts;
  reshaped_pts[0].second = 6.0;   // leg 0 pushed 6 m toward leg 2
  reshaped_pts[1].second = 6.0;
  const auto nominal = makePath(nominal_pts);
  const auto reshaped_dense = densify(makePath(reshaped_pts), 2.0);

  // Cursor mid leg 0 of the reshaped dense path: anchor ~(50, 6).
  const int dense_cursor = 25;
  const auto & anchor = reshaped_dense[dense_cursor].pose.position;
  ASSERT_NEAR(anchor.y, 6.0, 1e-9);
  ASSERT_LT(std::abs(anchor.x - 50.0), 4.0);
  // Sanity: the anchor really is nearer the wrong leg in space.
  ASSERT_LT(pointToSegmentDistance(nominal, 2, anchor.x, anchor.y),
    pointToSegmentDistance(nominal, 0, anchor.x, anchor.y));

  bool fallback = true;
  const int mapped =
    mncpf::mapCursorToNewPath(reshaped_dense, dense_cursor, nominal, &fallback);
  EXPECT_EQ(mapped, 0);
  EXPECT_FALSE(fallback);
}

// A front-truncated same-goal re-issue (BT-pruned transit path) must keep the
// geometric position, not the index.
TEST(MapCursorToNewPath, TruncatedReissueKeepsGeometricPosition)
{
  const auto sparse = makePath(kZigzag);
  const auto dense = densify(sparse, 2.0);
  std::vector<PoseStamped> truncated(dense.begin() + 3, dense.end());

  const int dense_cursor = 10;
  const auto & anchor = dense[dense_cursor].pose.position;
  const int mapped = mncpf::mapCursorToNewPath(dense, dense_cursor, truncated);

  // Mapped segment start must be geometrically at the anchor (one station).
  const auto & p = truncated[mapped].pose.position;
  EXPECT_LT(std::hypot(p.x - anchor.x, p.y - anchor.y), 2.5);
  EXPECT_NE(mapped, dense_cursor);  // index-preservation would be wrong here
}

// Old cursor past the new path's extent: clamp to the last traversable
// segment, never the done-sentinel (size-1).
TEST(MapCursorToNewPath, PastEndClampsToLastTraversableSegment)
{
  const auto sparse = makePath(kZigzag);
  const auto dense = densify(sparse, 2.0);

  const int at_end = static_cast<int>(dense.size()) - 2;
  const int mapped = mncpf::mapCursorToNewPath(dense, at_end, sparse);
  EXPECT_EQ(mapped, static_cast<int>(sparse.size()) - 2);

  // Done-sentinel input (== size-1) clamps to the last traversable segment
  // before anchoring, and the output stays traversable.
  const int sentinel = static_cast<int>(dense.size()) - 1;
  const int mapped2 = mncpf::mapCursorToNewPath(dense, sentinel, sparse);
  EXPECT_EQ(mapped2, static_cast<int>(sparse.size()) - 2);
}

TEST(MapCursorToNewPath, DegenerateInputsReturnZero)
{
  const auto sparse = makePath(kZigzag);
  const std::vector<PoseStamped> empty;
  const auto single = makePath({{1.0, 2.0}});

  EXPECT_EQ(mncpf::mapCursorToNewPath(sparse, 3, empty), 0);
  EXPECT_EQ(mncpf::mapCursorToNewPath(sparse, 3, single), 0);
  EXPECT_EQ(mncpf::mapCursorToNewPath(empty, 3, sparse), 0);
  EXPECT_EQ(mncpf::mapCursorToNewPath(single, 0, sparse), 0);
}

// Identical representation: fast path keeps the cursor for every traversable
// index (and clamps a done-sentinel input into range).
TEST(MapCursorToNewPath, IdenticalPathKeepsCursor)
{
  const auto sparse = makePath(kZigzag);
  for (int i = 0; i < static_cast<int>(sparse.size()) - 1; ++i) {
    EXPECT_EQ(mncpf::mapCursorToNewPath(sparse, i, sparse), i);
  }
  EXPECT_EQ(
    mncpf::mapCursorToNewPath(sparse, static_cast<int>(sparse.size()) - 1, sparse),
    static_cast<int>(sparse.size()) - 2);
}

// Anti-snap-back (#250 property): under forward progress with alternating
// representations, the mapped cursor never moves backward at leg granularity.
TEST(MapCursorToNewPath, AlternatingReissuesNeverMoveBackward)
{
  const auto sparse = makePath(kZigzag);
  const auto dense = densify(sparse, 2.0);

  int prev_leg = 0;
  for (int dense_cursor = 0; dense_cursor < static_cast<int>(dense.size()) - 1;
    dense_cursor += 5)
  {
    const int leg = mncpf::mapCursorToNewPath(dense, dense_cursor, sparse);
    EXPECT_GE(leg, prev_leg) << "dense cursor " << dense_cursor;
    prev_leg = std::max(prev_leg, leg);
  }
  EXPECT_EQ(prev_leg, static_cast<int>(sparse.size()) - 2);
}

// Purely geometric reference-step bound (no reliance on the #66 slew limiter,
// which defaults to off): for an on-path pose mid-leg, the cross-track
// distance to the tracked segment changes by < 0.5 m across a dense<->sparse
// flip.
TEST(MapCursorToNewPath, CrossTrackReferenceStepBoundedAcrossFlip)
{
  const auto sparse = makePath(kZigzag);
  const auto dense = densify(sparse, 2.0);

  const int dense_cursor = 40;  // mid leg 0
  const auto & boat = dense[dense_cursor].pose.position;

  const double xtk_dense =
    pointToSegmentDistance(dense, dense_cursor, boat.x, boat.y);
  const int mapped = mncpf::mapCursorToNewPath(dense, dense_cursor, sparse);
  const double xtk_sparse = pointToSegmentDistance(sparse, mapped, boat.x, boat.y);

  EXPECT_LT(std::abs(xtk_sparse - xtk_dense), 0.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
