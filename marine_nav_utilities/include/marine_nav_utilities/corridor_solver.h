#ifndef MARINE_NAV_UTILITIES_CORRIDOR_SOLVER_H
#define MARINE_NAV_UTILITIES_CORRIDOR_SOLVER_H

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace marine_nav_utilities
{

// Tunables for the corridor solver. Cross-track offset `d` is the cross-track
// error by construction, so the XTE term is analytic (w_xte * d^2) and never
// shares the costmap's cost budget. See unh_marine_navigation #30 / #59.
struct CorridorParams
{
  double max_xte = 6.0;            // corridor half-width (m); also the give-up bound
  double lateral_step = 0.5;       // cross-track offset resolution (m)
  double w_xte = 1.0;              // restoring-spring weight on d^2
  double w_obs = 0.02;             // weight on graded obstacle cost [0..252]
  double w_smooth = 2.0;           // weight on (d_i - d_{i-1})^2 between stations
  double w_temporal = 0.0;         // weight on (d - prev_tick_d)^2 (chatter damping)
  double max_lateral_rate = 1.0;   // max |d_i - d_{i-1}| between adjacent stations (m)
  double lethal_cost = 253.0;      // sampled cost >= this is impassable (INSCRIBED/LETHAL)
};

// A 2D centreline station: position + unit left-normal, both in the path frame.
struct Station
{
  double x = 0.0;
  double y = 0.0;
  double nx = 0.0;   // unit left-normal x (90deg CCW from tangent)
  double ny = 0.0;   // unit left-normal y
  double yaw = 0.0;  // tangent heading
};

// Build the symmetric set of candidate lateral offsets, guaranteed to include
// exactly 0.0 at the centre index. e.g. max_xte=1.0, step=0.5 -> {-1,-0.5,0,0.5,1}.
std::vector<double> makeLateralOffsets(double max_xte, double lateral_step);

// Pure corridor dynamic program — no ROS, unit-testable with a hand-built cost
// matrix. `obstacle_costs[i][j]` is the sampled costmap cost at station i,
// offset column j (must match `offsets` size); a value >= params.lethal_cost is
// impassable. `offsets` is the output of makeLateralOffsets (centre == 0).
// `prev_offsets` feeds the temporal term (empty => treated as all-zero, term off).
// Stations in [active_begin, active_end) are optimised; the rest, plus the two
// active endpoints themselves, are pinned to d=0 so the result is a true detour
// that re-anchors to the line. Returns the chosen offset per station, or
// std::nullopt when no finite-cost path exists (corridor blocked).
std::optional<std::vector<double>> solveCorridorOffsets(
  const std::vector<std::vector<double>> & obstacle_costs,
  const std::vector<double> & offsets,
  const CorridorParams & params,
  const std::vector<double> & prev_offsets,
  std::size_t active_begin,
  std::size_t active_end);

// Resample a pose polyline into stations spaced ~`step` metres along arclength,
// carrying the per-segment tangent and left-normal. Fewer than 2 input poses
// yields an empty result.
std::vector<Station> resampleStations(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses, double step);

// Plan corridor offsets for a survey line given a costmap sampler — the
// node-free core of the controller-layer avoider (#59). Encapsulates:
//   * nearest-station selection for the robot at (robot_x, robot_y) (path frame),
//   * the anchor-behind-boat active range (the #59 fix: the detour's near anchor
//     is pinned to d=0 `anchor_behind_distance` metres *behind* the boat, so the
//     boat itself rides the deviation instead of being yanked back to the line),
//   * cost-matrix sampling, and the DP solve.
// `sample(px, py)` returns the costmap cost at a path-frame point, or any
// NEGATIVE value to mean out-of-window / impassable. Returns the per-station
// offset for every station, or std::nullopt when the line is out of the costmap
// window, has no interior to deviate, or the corridor is blocked. `prev_offsets`
// feeds the temporal term (size must equal stations.size() to take effect).
std::optional<std::vector<double>> planCorridorOffsets(
  const std::vector<Station> & stations,
  const std::function<double(double, double)> & sample,
  const CorridorParams & params,
  double station_step,
  double anchor_behind_distance,
  double robot_x,
  double robot_y,
  const std::vector<double> & prev_offsets);

}  // namespace marine_nav_utilities

#endif  // MARINE_NAV_UTILITIES_CORRIDOR_SOLVER_H
