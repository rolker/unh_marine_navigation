#ifndef MARINE_NAV_CA_SAFETY_CA_SAFETY_H
#define MARINE_NAV_CA_SAFETY_CA_SAFETY_H

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace marine_nav_ca_safety
{

/// A tunable is usable only if finite and positive. A NaN would otherwise slip
/// past a bare `<= 0` test into arithmetic/clamp (undefined behavior). Shared by
/// the pure logic below and the node's parameter validation.
inline bool isFinitePositive(double v)
{
  return std::isfinite(v) && v > 0.0;
}

/// Replace a non-finite command component with 0 so NaN/inf from upstream can
/// never reach the helm. (A zeroed command coasts to a stop — safe.)
inline double finiteOrZero(double v)
{
  return std::isfinite(v) ? v : 0.0;
}

/// A 2D obstacle point expressed in the node's computation (base) frame:
/// x forward, y left, per REP-103.
struct Point2
{
  double x{0.0};
  double y{0.0};
};

/// A planar velocity command (the subset of a Twist this node modulates).
struct Twist2
{
  double linear_x{0.0};
  double angular_z{0.0};
};

/// Geometry + behavior tunables. Validated by the node before use; the pure
/// functions additionally guard against a misordered slowdown length range so a
/// bad live update can never trigger undefined behavior.
struct SafetyParams
{
  double ttc_time_constant{4.0};     // s   — slowdown leading edge = speed * this + min
  double slowdown_min_length{5.0};   // m
  double slowdown_max_length{25.0};  // m
  double slowdown_speed_floor{0.1};  // m/s — forward speed kept during slowdown (yaw authority)
  double slowdown_width{6.0};        // m   — full lateral width of the slowdown corridor
  double stop_length{5.0};           // m
  double stop_width{4.0};            // m
  double reverse_speed{0.5};         // m/s — reverse-brake setpoint magnitude
  bool cancel_yaw_during_reverse{true};
};

/// Which safety zone the nearest obstacle falls in.
enum class Zone
{
  Clear,
  Slowdown,
  Stop
};

/// Speed-scaled slowdown leading-edge distance (time-to-collision form):
/// `clamp(speed * T + min, min, max)`. Negative speed is treated as 0 (the boat
/// is not closing forward). Robust to a misordered [min, max].
inline double slowdownLength(double speed, const SafetyParams & p)
{
  const double lo = p.slowdown_min_length;
  const double hi = std::max(p.slowdown_max_length, lo);
  const double len = std::max(0.0, speed) * p.ttc_time_constant + lo;
  return std::clamp(len, lo, hi);
}

/// Nearest forward-sector obstacle range: the smallest x over points inside the
/// centered forward box `[0, max_x] x [-width/2, +width/2]`. Returns +inf when no
/// point qualifies. Non-finite points are ignored.
inline double nearestForwardRange(const std::vector<Point2> & pts, double width, double max_x)
{
  double nearest = std::numeric_limits<double>::infinity();
  const double half = 0.5 * width;
  for (const auto & pt : pts) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
      continue;
    }
    if (pt.x >= 0.0 && pt.x <= max_x && std::abs(pt.y) <= half) {
      nearest = std::min(nearest, pt.x);
    }
  }
  return nearest;
}

/// Classify the active zone. `stop_range` is the nearest range within the stop
/// box; `slow_range` the nearest within the slowdown corridor (already bounded by
/// `slowdown_len`). Stop dominates slowdown.
inline Zone classify(double stop_range, double slow_range, double slowdown_len)
{
  if (std::isfinite(stop_range)) {
    return Zone::Stop;
  }
  if (std::isfinite(slow_range) && slow_range <= slowdown_len) {
    return Zone::Slowdown;
  }
  return Zone::Clear;
}

/// Forward-speed scale in [0, 1]: 1 at/beyond `slowdown_len`, ramping linearly to
/// 0 at the stop boundary. Degenerate range (`slowdown_len <= stop_len`) yields a
/// hard step at `stop_len`.
inline double slowdownScale(double range, double slowdown_len, double stop_len)
{
  if (!(slowdown_len > stop_len)) {
    return (range <= stop_len) ? 0.0 : 1.0;
  }
  const double t = (range - stop_len) / (slowdown_len - stop_len);
  return std::clamp(t, 0.0, 1.0);
}

/// Yaw-preserving slowdown: reduce a *positive* forward speed toward the floor,
/// never speeding up, never below the floor (so the boat keeps the thrust it
/// needs to yaw on a vectored hull). Yaw is passed through untouched. A
/// non-positive input linear_x is left alone (the node isn't driving forward).
inline Twist2 applySlowdown(const Twist2 & in, double scale, double speed_floor)
{
  Twist2 out = in;  // angular_z preserved
  if (in.linear_x > 0.0) {
    const double floor = std::max(0.0, speed_floor);
    const double scaled = in.linear_x * std::clamp(scale, 0.0, 1.0);
    out.linear_x = std::min(in.linear_x, std::max(scaled, floor));
  }
  return out;
}

/// Reverse-assisted stop setpoint. While the boat still has forward way on
/// (`measured_speed > stop_speed_eps`) and reversing is still permitted by the
/// node's distance/duration backstop (`reverse_allowed`), command reverse thrust
/// to brake; otherwise hold zero. Yaw is cancelled (straight brake) unless
/// pass-through is explicitly enabled.
inline Twist2 applyStop(
  const Twist2 & in, double measured_speed, bool reverse_allowed,
  const SafetyParams & p, double stop_speed_eps)
{
  Twist2 out;
  const bool still_moving = measured_speed > stop_speed_eps;
  out.linear_x = (reverse_allowed && still_moving) ? -std::abs(p.reverse_speed) : 0.0;
  out.angular_z = p.cancel_yaw_during_reverse ? 0.0 : in.angular_z;
  return out;
}

/// Reverse-brake backstop: is reverse still permitted? Bounded by elapsed time
/// always, and by traveled distance only when a start pose was captured
/// (`have_distance`). Both limits are evaluated against caller-supplied measured
/// quantities; the time bound is the odom-independent guarantee.
inline bool reverseAllowed(
  double elapsed_s, double duration_limit_s,
  bool have_distance, double distance_m, double distance_limit_m)
{
  if (!(elapsed_s < duration_limit_s)) {
    return false;
  }
  if (have_distance && !(distance_m < distance_limit_m)) {
    return false;
  }
  return true;
}

/// A reverse-brake episode ends (and its backstop timers may reset) only after
/// the Stop zone has been gone for longer than the debounce window. This makes
/// the backstop cumulative across transient declassification (a flickering cloud
/// must not keep restarting the timer and so permit unbounded reverse).
inline bool reverseEpisodeEnded(double since_last_stop_s, double clear_debounce_s)
{
  return since_last_stop_s > clear_debounce_s;
}

/// Corners of a centered forward box `[0, length] x [-width/2, +width/2]`, ordered
/// for a closed polygon (far-left, far-right, near-right, near-left). Used for the
/// CAMP zone-visualization polygons.
inline std::array<Point2, 4> forwardBoxCorners(double length, double width)
{
  const double h = 0.5 * width;
  return {Point2{length, h}, Point2{length, -h}, Point2{0.0, -h}, Point2{0.0, h}};
}

}  // namespace marine_nav_ca_safety

#endif  // MARINE_NAV_CA_SAFETY_CA_SAFETY_H
