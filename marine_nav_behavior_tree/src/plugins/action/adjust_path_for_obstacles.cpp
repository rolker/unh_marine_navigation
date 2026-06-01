#include "marine_nav_behavior_tree/plugins/action/adjust_path_for_obstacles.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2/LinearMath/Quaternion.h>  // NOLINT(build/include_order)
#include <tf2/LinearMath/Transform.h>  // NOLINT(build/include_order)
#include <tf2/LinearMath/Vector3.h>  // NOLINT(build/include_order)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/robot_utils.hpp"

namespace marine_nav_behavior_tree
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
// Below this peak |d| (m) the reshaped path is indistinguishable from nominal;
// pass the nominal path through unchanged so clear water stays byte-identical
// to today's behaviour (no spurious FollowPath preemption).
constexpr double kDeviationEpsilon = 0.05;
// nav2_msgs/Costmap raw values: 255 == NO_INFORMATION (unobserved). Treat as
// free — for a survey, unobserved cells are open water we simply haven't swept.
constexpr uint8_t kNoInformation = 255;
// Sampling a point outside the rolling costmap window: we can't vouch for it,
// so forbid deviating there (impassable), but a centreline miss just bounds the
// active range.
constexpr double kOutOfBounds = -1.0;
// Marker namespace for the operator-feedback MarkerArray (CAMP auto-discovers it).
constexpr char kVizNamespace[] = "survey_obstacle_avoidance";

// Live tunables are declared as ROS params `survey_avoidance.<port>` on the
// bt_navigator node (grouped so rqt_reconfigure clusters them), seeded from the
// matching BT port on first tick and authoritative thereafter. The suffix here
// MUST match the BT port name (the seed is read via getInput(suffix)).
constexpr char kParamNs[] = "survey_avoidance.";
struct ParamSeed
{
  const char * name;     // BT port name == param suffix
  double fallback;       // used if the port itself is unset
  const char * description;
};
constexpr ParamSeed kParamSeeds[] = {
  {"max_deviation", 6.0, "Max metres the line may bend off-course; also the give-up bound."},
  {"along_track_spacing", 2.0, "Waypoint spacing along the line (m)."},
  {"lateral_resolution", 0.5, "Cross-track search resolution (m)."},
  {"line_following_weight", 1.0, "Higher = hug the survey line more strongly."},
  {"obstacle_avoidance_weight", 0.02, "Higher = deviate more strongly around obstacles."},
  {"smoothness_weight", 2.0, "Higher = smoother detours (penalise sharp lateral changes)."},
  {"chatter_damping_weight", 0.0, "Higher = steadier path tick-to-tick (anti-chatter)."},
  {"max_lateral_change", 1.0, "Max cross-track change between adjacent waypoints (m)."},
  {"avoid_speed", 0.0, "Speed (m/s) through the manoeuvre via per-pose stamps; 0 = controller default."},
};
// Empty stand-in for the temporal term's "previous offsets" when there is no
// matching prior tick — an lvalue, so the DP-input ref binds without a per-tick copy.
const std::vector<double> kEmptyOffsets;
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

void applyAvoidanceSlowdown(
  nav_msgs::msg::Path & path, const std::vector<double> & offsets_d,
  double avoid_speed, double deviation_epsilon)
{
  const std::size_t n = path.poses.size();
  if (avoid_speed <= 0.0 || n != offsets_d.size() || n < 2) {
    return;
  }

  // Contiguous deviating run [first, last].
  std::size_t first = n;
  std::size_t last = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (std::abs(offsets_d[i]) >= deviation_epsilon) {
      if (first == n) {
        first = i;
      }
      last = i;
    }
  }
  if (first >= last) {
    return;  // need >= 2 poses to form a stamped segment
  }

  // Non-zero base (1 s) so the controller treats the stamps as valid; only the
  // deltas carry meaning, and they are derived from geometry + speed (no wall
  // clock), so a held detour yields identical stamps tick-to-tick.
  constexpr int64_t kBaseNs = 1000000000LL;
  constexpr int64_t kNsPerS = 1000000000LL;
  double cum_s = 0.0;
  for (std::size_t i = first; i <= last; ++i) {
    if (i > first) {
      const double d = std::hypot(
        path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
        path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
      cum_s += d / avoid_speed;
    }
    const int64_t ns = kBaseNs + static_cast<int64_t>(cum_s * 1e9);
    path.poses[i].header.stamp.sec = static_cast<int32_t>(ns / kNsPerS);
    path.poses[i].header.stamp.nanosec = static_cast<uint32_t>(ns % kNsPerS);
  }
}

namespace
{
// Build the operator-feedback overlay for a deviating tick: the nominal survey
// line (faint), the adjusted path the boat will follow (bright), the deviating
// stretch highlighted (red), and an "AVOIDING" flag at the peak deviation. All
// in the path frame; CAMP auto-discovers any visualization_msgs/MarkerArray.
visualization_msgs::msg::MarkerArray buildAvoidanceMarkers(
  const rclcpp::Time & stamp, const std::string & frame,
  const nav_msgs::msg::Path & nominal,
  const std::vector<Station> & stations,
  const std::vector<double> & offsets_d)
{
  visualization_msgs::msg::MarkerArray arr;
  const auto lifetime = rclcpp::Duration::from_seconds(2.0);

  auto base = [&](int id, int32_t type, double width,
      double r, double g, double b, double a) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = stamp;
      m.ns = kVizNamespace;
      m.id = id;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = width;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = a;
      m.lifetime = lifetime;
      return m;
    };

  auto adjusted_point = [&](std::size_t i) {
      geometry_msgs::msg::Point pt;
      pt.x = stations[i].x + offsets_d[i] * stations[i].nx;
      pt.y = stations[i].y + offsets_d[i] * stations[i].ny;
      return pt;
    };

  // 1) Nominal survey line (faint gray) — context for how far we deviated.
  auto nominal_m = base(0, visualization_msgs::msg::Marker::LINE_STRIP, 0.4,
      0.6, 0.6, 0.6, 0.4);
  for (const auto & p : nominal.poses) {
    nominal_m.points.push_back(p.pose.position);
  }
  arr.markers.push_back(nominal_m);

  // 2) Adjusted path the boat follows (bright cyan).
  auto adjusted_m = base(1, visualization_msgs::msg::Marker::LINE_STRIP, 1.0,
      0.0, 0.9, 1.0, 0.95);
  std::size_t peak_i = 0;
  double peak = 0.0;
  for (std::size_t i = 0; i < stations.size(); ++i) {
    adjusted_m.points.push_back(adjusted_point(i));
    if (std::abs(offsets_d[i]) > peak) {
      peak = std::abs(offsets_d[i]);
      peak_i = i;
    }
  }
  arr.markers.push_back(adjusted_m);

  // 3) Avoiding band: the deviating stretch(es), red and thick, drawn on top.
  // LINE_LIST (not LINE_STRIP) of consecutive deviating pairs, so two separate
  // deviating runs (e.g. two obstacles) aren't joined by a straight line across
  // the on-line gap between them.
  auto band_m = base(2, visualization_msgs::msg::Marker::LINE_LIST, 1.6,
      1.0, 0.15, 0.1, 0.9);
  for (std::size_t i = 0; i + 1 < stations.size(); ++i) {
    if (std::abs(offsets_d[i]) >= kDeviationEpsilon &&
      std::abs(offsets_d[i + 1]) >= kDeviationEpsilon)
    {
      band_m.points.push_back(adjusted_point(i));
      band_m.points.push_back(adjusted_point(i + 1));
    }
  }
  if (!band_m.points.empty()) {
    arr.markers.push_back(band_m);
  }

  // 4) "AVOIDING" flag at the peak-deviation point (red, view-facing text).
  auto text_m = base(3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.0,
      1.0, 0.2, 0.15, 1.0);
  text_m.scale.z = 3.0;  // text height (m)
  text_m.text = "AVOIDING";
  text_m.pose.position = adjusted_point(peak_i);
  arr.markers.push_back(text_m);

  return arr;
}
}  // namespace

AdjustPathForObstacles::AdjustPathForObstacles(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList AdjustPathForObstacles::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>(
      "nominal_path", "{nominal_survey_path}",
      "Nominal trackline to reshape around obstacles."),
    BT::OutputPort<nav_msgs::msg::Path>(
      "path", "{survey_path}",
      "Reshaped path for FollowPath (nominal pass-through when clear/blocked)."),
    BT::InputPort<std::string>(
      "costmap_topic", "local_costmap/costmap_raw",
      "nav2_msgs/Costmap topic supplying the obstacle field."),
    // Tunables. Each port seeds the live ROS param `survey_avoidance.<name>` on
    // first tick; the param is authoritative thereafter (rqt_reconfigure /
    // `ros2 param set`). See kParamSeeds.
    BT::InputPort<double>(
      "max_deviation", 6.0,
      "Max metres the line may bend off-course; also the give-up bound."),
    BT::InputPort<double>("along_track_spacing", 2.0, "Waypoint spacing along the line (m)."),
    BT::InputPort<double>("lateral_resolution", 0.5, "Cross-track search resolution (m)."),
    BT::InputPort<double>(
      "line_following_weight", 1.0, "Higher = hug the survey line more strongly."),
    BT::InputPort<double>(
      "obstacle_avoidance_weight", 0.02, "Higher = deviate more strongly around obstacles."),
    BT::InputPort<double>(
      "smoothness_weight", 2.0, "Higher = smoother detours (penalise sharp lateral changes)."),
    BT::InputPort<double>(
      "chatter_damping_weight", 0.0, "Higher = steadier path tick-to-tick (anti-chatter)."),
    BT::InputPort<double>(
      "max_lateral_change", 1.0, "Max cross-track change between adjacent waypoints (m)."),
    BT::InputPort<double>(
      "avoid_speed", 0.0,
      "Speed (m/s) through the avoidance manoeuvre (via per-pose timestamps); "
      "0 disables (controller default speed)."),
  };
}

BT::NodeStatus AdjustPathForObstacles::tick()
{
  auto nominal_bb = getInput<nav_msgs::msg::Path>("nominal_path");
  if (!nominal_bb) {
    throw BT::RuntimeError(name(), " missing required input [nominal_path]: ",
      nominal_bb.error());
  }
  const nav_msgs::msg::Path nominal = nominal_bb.value();

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Lazy-create the costmap subscription on first tick (the topic port and the
  // blackboard node are only reliably readable once the tree is wired). The
  // callback runs on the bt_navigator executor thread, distinct from this tick
  // thread — hence the mutex + shared_ptr swap. The lambda captures the shared
  // cache (not `this`), so a callback still in flight on the executor thread when
  // this node is destroyed (tree reload / shutdown) touches live state, not freed.
  if (!costmap_sub_) {
    costmap_topic_ = getInput<std::string>("costmap_topic").value_or(
      "local_costmap/costmap_raw");
    auto cache = costmap_cache_;
    costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_, rclcpp::QoS(1),
      [cache](nav2_msgs::msg::Costmap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cache->mutex);
        cache->costmap = msg;
      });
    viz_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "survey_obstacle_avoidance", rclcpp::QoS(1));
    // Expose every tunable as a live `survey_avoidance.<name>` ROS parameter on
    // the bt_navigator node, seeded from the matching BT port. The params are
    // authoritative thereafter, so the whole avoider tunes without redeploying
    // the BT (`ros2 param set <bt_navigator> survey_avoidance.<name> <value>` /
    // rqt_reconfigure). has_parameter guards re-declaration across tree reloads
    // and honours a params-yaml override.
    for (const auto & ps : kParamSeeds) {
      const std::string full = std::string(kParamNs) + ps.name;
      if (!node->has_parameter(full)) {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = ps.description;
        node->declare_parameter(
          full, getInput<double>(ps.name).value_or(ps.fallback), desc);
      }
    }
  }

  // Wipe the avoidance overlay once on the avoiding->clear transition (idle ticks
  // then publish nothing, rather than spamming a DELETEALL every loop).
  auto clear_viz = [&]() {
    if (was_avoiding_ && viz_pub_) {
      visualization_msgs::msg::MarkerArray arr;
      visualization_msgs::msg::Marker del;
      // Give the DELETEALL a valid header — some marker consumers (RViz/CAMP)
      // warn on or drop markers with an empty frame_id even for a delete.
      del.header.frame_id = config().blackboard->get<std::string>("robot_frame");
      del.header.stamp = node->now();
      del.ns = kVizNamespace;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
      viz_pub_->publish(arr);
    }
    was_avoiding_ = false;
  };

  auto passthrough = [&]() {
    clear_viz();
    setOutput("path", nominal);
    prev_offsets_.clear();
    return BT::NodeStatus::SUCCESS;
  };

  if (nominal.poses.size() < 2) {
    return passthrough();
  }

  // Snapshot the latest costmap (pointer copy under the lock; DP reads it lock-free).
  std::shared_ptr<nav2_msgs::msg::Costmap> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_cache_->mutex);
    costmap = costmap_cache_->costmap;
  }
  if (!costmap) {
    return passthrough();  // no costmap yet
  }

  // Guard the axis-aligned indexing assumptions before sampling: positive
  // resolution, non-empty grid, a data buffer matching size_x*size_y, and an
  // identity origin orientation. nav2's costmap_2d always emits such a grid, but
  // a malformed or rotated relayed source would otherwise misregister every
  // sample. Degrade to passthrough (reflex layer remains the backstop).
  {
    const auto & m = costmap->metadata;
    const auto & q = m.origin.orientation;
    const bool axis_aligned =
      std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6 &&
      std::abs(std::abs(q.w) - 1.0) < 1e-6;
    if (m.resolution <= 0.0 || m.size_x == 0 || m.size_y == 0 ||
      costmap->data.size() != static_cast<std::size_t>(m.size_x) * m.size_y ||
      !axis_aligned)
    {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "AdjustPathForObstacles: unusable costmap (resolution=%.3f size=%ux%u "
        "data=%zu axis_aligned=%d); passing nominal path.",
        m.resolution, m.size_x, m.size_y, costmap->data.size(),
        static_cast<int>(axis_aligned));
      return passthrough();
    }
  }

  // Read the live tunables from the survey_avoidance.* ROS params (declared +
  // seeded on first tick); the internal CorridorParams keep their terse math
  // names. get_p resolves `survey_avoidance.<suffix>`.
  auto get_p = [&](const char * suffix) {
      return node->get_parameter(std::string(kParamNs) + suffix).as_double();
    };
  const double station_step = get_p("along_track_spacing");
  CorridorParams params;
  params.max_xte = get_p("max_deviation");
  params.lateral_step = get_p("lateral_resolution");
  params.w_xte = get_p("line_following_weight");
  params.w_obs = get_p("obstacle_avoidance_weight");
  params.w_smooth = get_p("smoothness_weight");
  params.w_temporal = get_p("chatter_damping_weight");
  params.max_lateral_rate = get_p("max_lateral_change");

  const auto stations = resampleStations(nominal.poses, station_step);
  if (stations.size() < 3) {
    return passthrough();
  }
  const std::size_t n = stations.size();

  // Resolve the transform from the path frame into the costmap frame (one lookup,
  // reused for every sample). Identity when the frames already match.
  const std::string path_frame = nominal.header.frame_id.empty()
    ? nominal.poses.front().header.frame_id
    : nominal.header.frame_id;
  const std::string costmap_frame = costmap->header.frame_id;

  auto tf_buffer =
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  tf2::Transform to_costmap;
  try {
    const auto tf_msg = tf_buffer->lookupTransform(
      costmap_frame, path_frame, tf2::TimePointZero);
    tf2::fromMsg(tf_msg.transform, to_costmap);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "AdjustPathForObstacles: TF %s -> %s unavailable (%s); passing nominal path.",
      path_frame.c_str(), costmap_frame.c_str(), ex.what());
    return passthrough();
  }

  // Sample the costmap at a path-frame point; returns raw cost mapped so that
  // out-of-window -> impassable, NO_INFORMATION -> free, else the graded value.
  const auto & md = costmap->metadata;
  auto sample = [&](double px, double py) -> double {
    const tf2::Vector3 w = to_costmap * tf2::Vector3(px, py, 0.0);
    const int mx = static_cast<int>(
      std::floor((w.x() - md.origin.position.x) / md.resolution));
    const int my = static_cast<int>(
      std::floor((w.y() - md.origin.position.y) / md.resolution));
    if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(md.size_x) || my >= static_cast<int>(md.size_y))
    {
      return kOutOfBounds;
    }
    const uint8_t c = costmap->data[my * md.size_x + mx];
    if (c == kNoInformation) {
      return 0.0;
    }
    return static_cast<double>(c);
  };

  // Robot's station index, used to clip the active range to ahead-of-boat.
  // Without a pose we can't tell which stations are astern, and reshaping behind
  // the boat can tug a nearest-segment follower backward — so a pose-lookup
  // failure passes the nominal path through for this tick rather than guessing.
  std::size_t robot_station = 0;
  {
    auto robot_frame = config().blackboard->get<std::string>("robot_frame");
    geometry_msgs::msg::PoseStamped robot;
    if (!nav2_util::getCurrentPose(robot, *tf_buffer, path_frame, robot_frame, 0.1)) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "AdjustPathForObstacles: robot pose (%s->%s) unavailable; passing nominal path.",
        path_frame.c_str(), robot_frame.c_str());
      return passthrough();
    }
    double best_d = kInf;
    for (std::size_t i = 0; i < n; ++i) {
      const double d = std::hypot(
        stations[i].x - robot.pose.position.x,
        stations[i].y - robot.pose.position.y);
      if (d < best_d) {
        best_d = d;
        robot_station = i;
      }
    }
  }

  // Active range: contiguous in-window stations starting at/after the boat.
  std::size_t active_begin = n;
  for (std::size_t i = robot_station; i < n; ++i) {
    if (sample(stations[i].x, stations[i].y) != kOutOfBounds) {
      active_begin = i;
      break;
    }
  }
  if (active_begin >= n) {
    return passthrough();  // nothing of the line ahead is in the costmap window
  }
  std::size_t active_end = n;
  for (std::size_t i = active_begin + 1; i < n; ++i) {
    if (sample(stations[i].x, stations[i].y) == kOutOfBounds) {
      active_end = i;
      break;
    }
  }
  if (active_end - active_begin < 3) {
    return passthrough();  // no interior station to deviate
  }

  // Build the cost matrix (active rows sampled; inactive rows unused -> zeros).
  const auto offsets = makeLateralOffsets(params.max_xte, params.lateral_step);
  std::vector<std::vector<double>> costs(n, std::vector<double>(offsets.size(), 0.0));
  for (std::size_t i = active_begin; i < active_end; ++i) {
    for (std::size_t j = 0; j < offsets.size(); ++j) {
      const double px = stations[i].x + offsets[j] * stations[i].nx;
      const double py = stations[i].y + offsets[j] * stations[i].ny;
      const double s = sample(px, py);
      costs[i][j] = (s == kOutOfBounds) ? params.lethal_cost : s;
    }
  }

  const std::vector<double> & prev =
    (prev_offsets_.size() == n) ? prev_offsets_ : kEmptyOffsets;
  const auto solved = solveCorridorOffsets(
    costs, offsets, params, prev, active_begin, active_end);
  if (!solved) {
    return passthrough();  // corridor blocked -> degrade to reflex/RecoveryNode
  }

  double peak = 0.0;
  for (const double d : *solved) {
    peak = std::max(peak, std::abs(d));
  }
  prev_offsets_ = *solved;
  if (peak < kDeviationEpsilon) {
    clear_viz();  // back on the line: wipe any stale avoidance overlay
    setOutput("path", nominal);  // no meaningful deviation: keep nominal exactly
    return BT::NodeStatus::SUCCESS;
  }

  // Emit the reshaped path (path frame; zero outer stamp = "latest" per #23).
  nav_msgs::msg::Path out;
  out.header.frame_id = path_frame;
  out.header.stamp = builtin_interfaces::msg::Time();
  out.poses.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = path_frame;
    p.pose.position.x = stations[i].x + (*solved)[i] * stations[i].nx;
    p.pose.position.y = stations[i].y + (*solved)[i] * stations[i].ny;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, stations[i].yaw);
    p.pose.orientation = tf2::toMsg(q);
    out.poses.push_back(p);
  }

  // Optional slow-down through the manoeuvre (off by default): stamp the
  // deviating run so the controller commands the avoidance speed there. Read
  // from the live `survey_avoidance.avoid_speed` param (seeded from the port).
  applyAvoidanceSlowdown(out, *solved, get_p("avoid_speed"), kDeviationEpsilon);
  setOutput("path", out);

  // Operator feedback: show the deviation in CAMP (nominal line, adjusted path,
  // the avoiding band, and an "AVOIDING" flag). Published only here, on a real
  // deviation.
  if (viz_pub_) {
    viz_pub_->publish(
      buildAvoidanceMarkers(node->now(), path_frame, nominal, stations, *solved));
    was_avoiding_ = true;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace marine_nav_behavior_tree
