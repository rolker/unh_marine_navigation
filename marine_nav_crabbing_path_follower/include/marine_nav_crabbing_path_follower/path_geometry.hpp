#ifndef MARINE_NAV_CRABBING_PATH_FOLLOWER__PATH_GEOMETRY_HPP_
#define MARINE_NAV_CRABBING_PATH_FOLLOWER__PATH_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
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
  const double v = std::max(target_speed, v_min);
  return crab_angle_deg * gain_ref_speed / v;
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
