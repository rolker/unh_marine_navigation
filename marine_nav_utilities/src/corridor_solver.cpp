#include "marine_nav_utilities/corridor_solver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace marine_nav_utilities
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

std::vector<double> makeLateralOffsets(double max_xte, double lateral_step)
{
  std::vector<double> offsets;
  if (lateral_step <= 0.0 || max_xte < 0.0) {
    offsets.push_back(0.0);
    return offsets;
  }
  // Floor (not round) so the extreme offset n_each*lateral_step never exceeds
  // max_xte — max_xte is a hard corridor half-width, and with live params it can
  // be set to any value, not just a multiple of lateral_step. The +1e-9 guards an
  // exact multiple from under-flooring on fp error. A corridor narrower than one
  // lateral_step yields only {0.0} (no deviation) — by design: lower
  // lateral_resolution to use a sub-step corridor.
  const int n_each = static_cast<int>(std::floor(max_xte / lateral_step + 1e-9));
  for (int k = -n_each; k <= n_each; ++k) {
    offsets.push_back(k * lateral_step);
  }
  // Guarantee an exact 0.0 centre even if fp nudged it.
  if (n_each >= 0) {
    offsets[n_each] = 0.0;
  }
  return offsets;
}

std::optional<std::vector<double>> solveCorridorOffsets(
  const std::vector<std::vector<double>> & obstacle_costs,
  const std::vector<double> & offsets,
  const CorridorParams & params,
  const std::vector<double> & prev_offsets,
  std::size_t active_begin,
  std::size_t active_end)
{
  const std::size_t n = obstacle_costs.size();
  const std::size_t m = offsets.size();
  if (n == 0 || m == 0) {
    return std::nullopt;
  }

  // Centre column (d == 0) — the pin target for inactive stations and anchors.
  std::size_t zero_col = 0;
  for (std::size_t j = 0; j < m; ++j) {
    if (std::abs(offsets[j]) < std::abs(offsets[zero_col])) {
      zero_col = j;
    }
  }

  const bool use_temporal =
    params.w_temporal > 0.0 && prev_offsets.size() == n;

  // A station is "free to choose" only inside [active_begin, active_end), and
  // never at the two active endpoints (anchored to d=0 so the detour returns).
  auto pinned_to_zero = [&](std::size_t i) {
    if (i < active_begin || i + 1 > active_end) {
      return true;  // outside the active range
    }
    return i == active_begin || i + 1 == active_end;  // the anchored endpoints
  };

  // node_cost(i, j): finite only for admissible (station, offset) pairs.
  auto node_cost = [&](std::size_t i, std::size_t j) -> double {
    if (pinned_to_zero(i)) {
      if (j != zero_col) {
        return kInf;
      }
      // A pinned station is forced to d=0, but it must still respect a lethal
      // cell there — otherwise an anchor re-joining the line on top of an
      // obstacle would be reported as a clear corridor. (Out-of-active-range
      // rows hold 0.0 and are unaffected; the active endpoints carry real
      // sampled cost.) Lethal anchor => no finite path => caller passes nominal
      // through and the reflex layer is the backstop.
      return obstacle_costs[i][zero_col] >= params.lethal_cost ? kInf : 0.0;
    }
    const double cost = obstacle_costs[i][j];
    if (cost >= params.lethal_cost) {
      return kInf;
    }
    double c = params.w_xte * offsets[j] * offsets[j] + params.w_obs * cost;
    if (use_temporal) {
      const double dt = offsets[j] - prev_offsets[i];
      c += params.w_temporal * dt * dt;
    }
    return c;
  };

  // Forward DP. best[j] = min cost to reach station i at offset column j.
  std::vector<double> best(m, kInf);
  std::vector<std::vector<int>> parent(n, std::vector<int>(m, -1));

  for (std::size_t j = 0; j < m; ++j) {
    best[j] = node_cost(0, j);
  }

  for (std::size_t i = 1; i < n; ++i) {
    std::vector<double> cur(m, kInf);
    for (std::size_t j = 0; j < m; ++j) {
      const double nc = node_cost(i, j);
      if (!std::isfinite(nc)) {
        continue;
      }
      for (std::size_t k = 0; k < m; ++k) {
        if (!std::isfinite(best[k])) {
          continue;
        }
        const double step = offsets[j] - offsets[k];
        if (std::abs(step) > params.max_lateral_rate + 1e-9) {
          continue;  // exceeds the per-station lateral-rate limit
        }
        const double trans = params.w_smooth * step * step;
        const double total = best[k] + trans + nc;
        if (total < cur[j]) {
          cur[j] = total;
          parent[i][j] = static_cast<int>(k);
        }
      }
    }
    best.swap(cur);
  }

  // Best terminal column (the last station is pinned, so this is zero_col).
  std::size_t end_col = 0;
  double end_cost = kInf;
  for (std::size_t j = 0; j < m; ++j) {
    if (best[j] < end_cost) {
      end_cost = best[j];
      end_col = j;
    }
  }
  if (!std::isfinite(end_cost)) {
    return std::nullopt;  // corridor blocked — no finite-cost path
  }

  std::vector<double> result(n, 0.0);
  std::size_t col = end_col;
  for (std::size_t ii = n; ii-- > 0;) {
    result[ii] = offsets[col];
    const int p = parent[ii][col];
    if (p < 0) {
      break;
    }
    col = static_cast<std::size_t>(p);
  }
  return result;
}

std::vector<Station> resampleStations(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses, double step)
{
  std::vector<Station> stations;
  if (poses.size() < 2 || step <= 0.0) {
    return stations;
  }

  // Collapse consecutive duplicate points.
  std::vector<std::array<double, 2>> pts;
  for (const auto & p : poses) {
    const double x = p.pose.position.x;
    const double y = p.pose.position.y;
    if (pts.empty() || std::hypot(x - pts.back()[0], y - pts.back()[1]) > 1e-6) {
      pts.push_back({x, y});
    }
  }
  if (pts.size() < 2) {
    return stations;
  }

  // Per-segment length + total.
  std::vector<double> seg_len(pts.size(), 0.0);  // seg_len[i] = |pts[i-1]→pts[i]|
  double total = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    seg_len[i] = std::hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    total += seg_len[i];
  }
  if (total <= 0.0) {
    return stations;
  }

  // Single forward walk: the sample arclength s is monotonically increasing, so
  // the active segment index only advances — O(segments + stations) overall
  // (vs a per-station rescan from the start). `seg_start` is the arclength at
  // pts[seg-1]; we stay on the last segment for s at/after `total`.
  std::size_t seg = 1;
  double seg_start = 0.0;
  auto station_at = [&](double s) -> Station {
    while (seg + 1 < pts.size() && s > seg_start + seg_len[seg]) {
      seg_start += seg_len[seg];
      ++seg;
    }
    const double dx = pts[seg][0] - pts[seg - 1][0];
    const double dy = pts[seg][1] - pts[seg - 1][1];
    const double len = seg_len[seg];
    const double t = std::clamp((s - seg_start) / len, 0.0, 1.0);
    const double tx = dx / len;
    const double ty = dy / len;
    Station st;
    st.x = pts[seg - 1][0] + t * dx;
    st.y = pts[seg - 1][1] + t * dy;
    st.nx = -ty;  // left normal (90deg CCW from tangent)
    st.ny = tx;
    st.yaw = std::atan2(ty, tx);
    return st;
  };

  for (double s = 0.0; s < total; s += step) {
    stations.push_back(station_at(s));
  }
  stations.push_back(station_at(total));  // always anchor the endpoint
  return stations;
}

std::optional<std::vector<double>> planCorridorOffsets(
  const std::vector<Station> & stations,
  const std::function<double(double, double)> & sample,
  const CorridorParams & params,
  double station_step,
  double anchor_behind_distance,
  double robot_x,
  double robot_y,
  const std::vector<double> & prev_offsets)
{
  const std::size_t n = stations.size();
  if (n < 3 || station_step <= 0.0 || !sample) {
    return std::nullopt;
  }

  // Robot's nearest station.
  std::size_t robot_station = 0;
  double best_d = kInf;
  for (std::size_t i = 0; i < n; ++i) {
    const double d = std::hypot(stations[i].x - robot_x, stations[i].y - robot_y);
    if (d < best_d) {
      best_d = d;
      robot_station = i;
    }
  }

  // Anchor the detour entry a fixed distance behind the boat (#59 fix).
  const int back = std::max(
    0, static_cast<int>(std::lround(anchor_behind_distance / station_step)));
  const std::size_t anchor_start =
    (robot_station > static_cast<std::size_t>(back)) ? robot_station - back : 0;

  // Active range: contiguous in-window stations from anchor_start (sample >= 0).
  std::size_t active_begin = n;
  for (std::size_t i = anchor_start; i < n; ++i) {
    if (sample(stations[i].x, stations[i].y) >= 0.0) {
      active_begin = i;
      break;
    }
  }
  if (active_begin >= n) {
    return std::nullopt;  // nothing of the line is in the costmap window
  }
  std::size_t active_end = n;
  for (std::size_t i = active_begin + 1; i < n; ++i) {
    if (sample(stations[i].x, stations[i].y) < 0.0) {
      active_end = i;
      break;
    }
  }
  if (active_end - active_begin < 3) {
    return std::nullopt;  // no interior station to deviate
  }

  // Build the cost matrix (active rows sampled at each candidate offset; a
  // negative sample == out-of-window == impassable for that cell).
  const auto offsets = makeLateralOffsets(params.max_xte, params.lateral_step);
  std::vector<std::vector<double>> costs(n, std::vector<double>(offsets.size(), 0.0));
  for (std::size_t i = active_begin; i < active_end; ++i) {
    for (std::size_t j = 0; j < offsets.size(); ++j) {
      const double px = stations[i].x + offsets[j] * stations[i].nx;
      const double py = stations[i].y + offsets[j] * stations[i].ny;
      const double s = sample(px, py);
      costs[i][j] = (s < 0.0) ? params.lethal_cost : s;
    }
  }

  const std::vector<double> empty;
  return solveCorridorOffsets(
    costs, offsets, params,
    (prev_offsets.size() == n) ? prev_offsets : empty,
    active_begin, active_end);
}

}  // namespace marine_nav_utilities
