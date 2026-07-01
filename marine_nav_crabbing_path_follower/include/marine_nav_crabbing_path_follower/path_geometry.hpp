#ifndef MARINE_NAV_CRABBING_PATH_FOLLOWER__PATH_GEOMETRY_HPP_
#define MARINE_NAV_CRABBING_PATH_FOLLOWER__PATH_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace marine_nav_crabbing_path_follower
{

/// Slew `current` toward `target`, advancing by at most `max_step`. A
/// non-positive `max_step` disables limiting (returns `target` unchanged), so a
/// zero rate is a clean "off". Used to rate-limit the cross-track error fed to
/// the cross-track PID (#66): a discontinuous reference step — a planner replan
/// or the avoidance decorator reshaping the line under the boat — is ramped in
/// instead of kicking the controller into an over-correction, while genuine
/// lateral drift (far slower than a sane rate) passes through untouched.
inline double slewToward(double current, double target, double max_step)
{
  if (!(max_step > 0.0)) {
    return target;
  }
  const double delta = target - current;
  return current + std::clamp(delta, -max_step, max_step);
}

/// Stateful slew-limit of the value (a cross-track error) fed to the PID (#66).
/// Holds `slewed`/`initialized` across control cycles. Branches, in order:
///   - first call after construction or a PID reset (`initialized` false): seed
///     to `raw` and pass it through (snap, don't ramp), so a post-gap resume
///     starts from the current error rather than a stale one;
///   - `rate <= 0`: limiting disabled — pass `raw` through (the historical
///     default); the held value is kept in step with `raw` (re-seeded), so a
///     later enable (rate 0 -> >0) starts cleanly from the current error;
///   - `dt_s <= 0` (a zero / duplicate-stamp cycle): HOLD the previous slewed
///     value — a replan landing on a zero-dt cycle must not leak the raw jump
///     through and defeat the limiter;
///   - otherwise: ramp `slewed` toward `raw` by at most `rate * dt_s`.
/// Returns the (possibly slewed) value to feed the controller.
inline double slewLimitError(
  double & slewed, bool & initialized, double raw, double rate, double dt_s)
{
  if (initialized && rate > 0.0) {
    if (dt_s > 0.0) {
      slewed = slewToward(slewed, raw, rate * dt_s);
    }
    return slewed;
  }
  slewed = raw;
  initialized = true;
  return raw;
}

/// Walk `lookahead` metres forward along a piecewise-linear path and return the
/// point reached — the pure-pursuit "look-ahead point".
///
/// The walk starts `start_offset` metres into segment `start_seg` (i.e. the
/// vehicle's projection onto that segment) and advances **forward only**. If the
/// look-ahead distance runs past the end of the path, the final path point (the
/// goal) is returned, so the follower converges onto the goal instead of
/// over-running it. Degenerate inputs (empty path, single point, out-of-range
/// start) return a sensible point rather than reading out of bounds.
inline geometry_msgs::msg::Point lookaheadPoint(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  int start_seg, double start_offset, double lookahead)
{
  geometry_msgs::msg::Point p;
  if (poses.empty()) {
    return p;
  }
  const int last = static_cast<int>(poses.size()) - 1;
  if (last == 0) {
    return poses[0].pose.position;
  }
  if (start_seg < 0) {
    start_seg = 0;
  }
  if (start_seg > last - 1) {
    return poses[last].pose.position;
  }

  double remaining = std::max(0.0, lookahead);
  for (int i = start_seg; i < last; ++i) {
    const auto & a = poses[i].pose.position;
    const auto & b = poses[i + 1].pose.position;
    const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
    const double offset = (i == start_seg) ? std::clamp(start_offset, 0.0, seg_len) : 0.0;
    const double avail = seg_len - offset;

    // Last segment, or the look-ahead lands within this segment: interpolate.
    if (remaining <= avail || i == last - 1) {
      const double along = std::min(offset + remaining, seg_len);
      const double f = (seg_len > 1e-9) ? along / seg_len : 0.0;
      p.x = a.x + f * (b.x - a.x);
      p.y = a.y + f * (b.y - a.y);
      p.z = a.z + f * (b.z - a.z);
      return p;
    }
    remaining -= avail;
  }
  return poses[last].pose.position;
}

/// Azimuth (radians, `atan2(dy, dx)`) of the path segment that CONTAINS the
/// pure-pursuit look-ahead point — the path tangent `lookahead` metres ahead.
///
/// Walks the path with the SAME forward-only traversal as `lookaheadPoint`
/// (same `start_seg`/`start_offset`/`lookahead` contract) but returns the
/// azimuth of the segment the look-ahead point lands on rather than the point
/// itself. Used as the follower's `base_heading` so cross-track correction is
/// left solely to the crab PID (no pure-pursuit double-counting; #91).
///
/// If the look-ahead runs past the end of the path — or the start segment is
/// already past the last segment — the FINAL segment's azimuth is returned
/// (mirroring `lookaheadPoint`'s goal-clamp), so overshoot yields the tangent
/// the boat is converging along, not a degenerate value. Only genuinely
/// degenerate inputs return `0.0`: an empty path, a single point (no segment),
/// or an all-coincident / zero-length path (`atan2(0, 0)` is naturally `0.0`).
inline double lookaheadSegmentAzimuth(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  int start_seg, double start_offset, double lookahead)
{
  if (poses.empty()) {
    return 0.0;
  }
  const int last = static_cast<int>(poses.size()) - 1;
  if (last == 0) {
    return 0.0;
  }
  if (start_seg < 0) {
    start_seg = 0;
  }
  if (start_seg > last - 1) {
    const auto & a = poses[last - 1].pose.position;
    const auto & b = poses[last].pose.position;
    return std::atan2(b.y - a.y, b.x - a.x);
  }

  double remaining = std::max(0.0, lookahead);
  for (int i = start_seg; i < last; ++i) {
    const auto & a = poses[i].pose.position;
    const auto & b = poses[i + 1].pose.position;
    const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
    const double offset = (i == start_seg) ? std::clamp(start_offset, 0.0, seg_len) : 0.0;
    const double avail = seg_len - offset;

    // Last segment, or the look-ahead lands within this segment: this is the
    // segment containing the look-ahead point.
    if (remaining <= avail || i == last - 1) {
      return std::atan2(b.y - a.y, b.x - a.x);
    }
    remaining -= avail;
  }
  // Unreachable (the loop always returns on i == last - 1); mirrors the
  // trailing final-segment return of `lookaheadPoint`.
  const auto & a = poses[last - 1].pose.position;
  const auto & b = poses[last].pose.position;
  return std::atan2(b.y - a.y, b.x - a.x);
}

/// Speed-normalize (gain-schedule) the cross-track PID output `crab_angle_deg`.
///
/// The outer cross-track loop has `ė ≈ v·sin(crab_angle) ≈ v·(p·e)`, so its
/// effective gain is proportional to the commanded speed `v` (#76). With fixed
/// PID gains the stability margin shrinks as speed rises, producing a
/// speed-dependent cross-track limit cycle (observed at Lake Massabesic,
/// unh_echoboats_project11#289). Scaling the PID output by
/// `gain_ref_speed / v` cancels that broadband plant gain, holding the
/// closed-loop response constant across speed: the controller behaves as if it
/// were always running at `gain_ref_speed`.
///
/// Contract:
///   - `gain_ref_speed <= 0`: disabled (the default) — returns `crab_angle_deg`
///     unchanged, so there is no behavior change until a platform opts in
///     (mirrors the `lookahead_time = 0` default-off idiom in this package).
///   - otherwise: returns `crab_angle_deg * gain_ref_speed / max(target_speed, v_min)`.
///     `v_min` floors the effective speed so creep / station-keep
///     (`target_speed → 0`) can't blow the gain up (or divide by zero). The
///     caller must pass a strictly-positive `v_min` (the parameter validator
///     enforces `> 0`); `target_speed` may be any value, including 0.
///
/// Pure (not a method) so it can be unit-tested across speeds with no ROS
/// scaffolding — the same reason `slewLimitError` / `lookaheadPoint` live here.
inline double gainScheduleScale(
  double crab_angle_deg, double gain_ref_speed, double v_min, double target_speed)
{
  if (!(gain_ref_speed > 0.0)) {
    return crab_angle_deg;
  }
  // A non-finite target_speed (NaN/Inf from a stale or wild estimate) would
  // propagate through std::max into the divisor and command NaN crab; treat it
  // as the floor so the result stays finite for finite crab_angle/gain_ref_speed/v_min.
  const double safe_target_speed = std::isfinite(target_speed) ? target_speed : v_min;
  const double v = std::max(safe_target_speed, v_min);
  return crab_angle_deg * gain_ref_speed / v;
}

/// Regulate the commanded surge on a turn by the magnitude of the crab angle (#87).
///
/// CrabbingPathFollower commands `linear.x = target_speed / cos(crab_angle)`:
/// when the cross-track PID drives a large crab angle on a turn, that division
/// *inflates* the surge exactly when the boat is turning hardest (+18% commanded
/// surge, +60% modelled current draw in turns vs. straights; field data
/// 2026-06-30, quad coulomb model). This factor pre-multiplies `target_speed` to
/// counter that inflation, slowing the boat in proportion to how hard it is
/// crabbing.
///
/// Contract:
///   - `max_crab_deg <= 0`: disabled (the default) — returns 1.0 always, so
///     there is no behavior change until a platform opts in (mirrors the
///     `gain_ref_speed = 0` default-off idiom in `gainScheduleScale`).
///   - otherwise: returns `clamp(1 - |crab_angle_deg| / max_crab_deg, min_factor, 1.0)`.
///     Zero crab -> 1.0 (no slowdown on straights); `|crab| >= max_crab_deg` ->
///     `min_factor` (maximum regulation). `min_factor` floors the slowdown so the
///     boat never stalls mid-turn; the caller passes it in `[0, 1]` (the
///     parameter validator enforces that band).
///   - A non-finite crab angle (NaN/Inf from a wild PID output) is treated as the
///     max-crab magnitude -> `min_factor`, so a wild input slows the boat rather
///     than propagating a non-finite factor into the commanded surge.
///
/// Pure (not a method) so it can be unit-tested across crab angles with no ROS
/// scaffolding — the same reason `gainScheduleScale` / `slewLimitError` /
/// `lookaheadPoint` live here.
inline double turnSpeedFactor(
  double crab_angle_deg, double max_crab_deg, double min_factor)
{
  if (!(max_crab_deg > 0.0)) {
    return 1.0;
  }
  if (!std::isfinite(crab_angle_deg)) {
    return min_factor;
  }
  const double frac = std::abs(crab_angle_deg) / max_crab_deg;
  return std::clamp(1.0 - frac, min_factor, 1.0);
}

/// Radius of the circle circumscribing the triangle a-b-c — the local radius of
/// curvature of the path passing through those three points (#89).
///
/// Formula: R = |AB|·|BC|·|CA| / (4·Area), where Area is the (unsigned) area of
/// triangle abc and |AB| etc. are the side lengths. The twice-signed-area is the
/// 2D cross product of AB and BC, so 4·Area = 2·|AB × BC|.
///
/// Returns `+infinity` for every degenerate input — collinear points (zero
/// area → a straight line, curvature 0), coincident points (fewer than three
/// distinct points, so no circle is defined), or any non-finite coordinate. A
/// single `!isfinite` guard on the result covers all three: collinear/coincident
/// drive the divisor to 0 (→ inf or 0/0 → NaN), and a NaN coordinate propagates
/// to a NaN result; both map to infinity. Infinity is the "straight, no
/// curvature" sentinel that `curvatureSpeedFactor` turns into no slowdown — the
/// safe default for geometry we can't interpret.
///
/// Pure (not a method) so it can be unit-tested with no ROS scaffolding — the
/// same reason `turnSpeedFactor` / `lookaheadPoint` live here.
inline double circumscribedRadius(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b,
  const geometry_msgs::msg::Point & c)
{
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double bcx = c.x - b.x;
  const double bcy = c.y - b.y;
  const double cax = a.x - c.x;
  const double cay = a.y - c.y;
  const double ab = std::hypot(abx, aby);
  const double bc = std::hypot(bcx, bcy);
  const double ca = std::hypot(cax, cay);
  // Twice the signed triangle area (the 2D cross product of AB and BC); its
  // magnitude is 2·Area, so 4·Area == 2·|cross|.
  const double cross = abx * bcy - aby * bcx;
  const double radius = (ab * bc * ca) / (2.0 * std::abs(cross));
  if (!std::isfinite(radius)) {
    return std::numeric_limits<double>::infinity();
  }
  return radius;
}

/// Anticipatory turn-speed factor from the local path radius of curvature (#89).
///
/// The reactive `turnSpeedFactor` slows the boat in proportion to the crab angle
/// it is *already* holding — a beat late into a turn. This factor is the
/// anticipatory half: given the circumscribed radius of the path geometry ahead
/// (see `circumscribedRadius`), it slows the boat *before* the apex of a tight
/// bend. The caller composes the two via `min()`.
///
/// Contract:
///   - `min_radius <= 0`: disabled (the default) — returns 1.0 always, so there
///     is no behavior change until a platform opts in (mirrors the
///     `turn_speed_max_crab_deg = 0` default-off idiom in `turnSpeedFactor`).
///   - `radius` non-finite (a straight/degenerate fit) or `radius >= min_radius`
///     (a gentle bend): returns 1.0 (no slowdown).
///   - otherwise (a tight turn, `radius < min_radius`): returns
///     `clamp(radius / min_radius, min_factor, 1.0)` — the tighter the turn the
///     lower the factor, floored at `min_factor` so the boat never stalls
///     mid-turn. `min_factor` is the SAME floor `turnSpeedFactor` uses (the two
///     regulators share one "slowest I'll go in a turn" knob).
///
/// Pure (not a method) so it can be unit-tested across radii with no ROS
/// scaffolding — the same reason `circumscribedRadius` / `turnSpeedFactor` live
/// here.
inline double curvatureSpeedFactor(
  double radius, double min_radius, double min_factor)
{
  if (!(min_radius > 0.0)) {
    return 1.0;
  }
  if (!std::isfinite(radius) || radius >= min_radius) {
    return 1.0;
  }
  return std::clamp(radius / min_radius, min_factor, 1.0);
}

/// Signed along-track distance of point `p` projected onto the segment a->b,
/// measured from `a`. Negative means `p` is behind the segment start. A
/// degenerate (zero-length) segment returns 0.
inline double alongTrackProjection(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b,
  const geometry_msgs::msg::Point & p)
{
  const double sx = b.x - a.x;
  const double sy = b.y - a.y;
  const double seg_len = std::hypot(sx, sy);
  if (seg_len < 1e-9) {
    return 0.0;
  }
  return ((p.x - a.x) * sx + (p.y - a.y) * sy) / seg_len;
}

}  // namespace marine_nav_crabbing_path_follower

#endif  // MARINE_NAV_CRABBING_PATH_FOLLOWER__PATH_GEOMETRY_HPP_
