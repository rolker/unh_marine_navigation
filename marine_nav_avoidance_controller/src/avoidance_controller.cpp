#include "marine_nav_avoidance_controller/avoidance_controller.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>  // NOLINT(build/include_order)
#include <tf2/LinearMath/Transform.h>  // NOLINT(build/include_order)
#include <tf2/LinearMath/Vector3.h>  // NOLINT(build/include_order)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace marine_nav_avoidance_controller
{

namespace
{
// Below this peak |d| (m) the reshaped path is indistinguishable from nominal.
constexpr double kDeviationEpsilon = 0.05;
// Sampling outside the rolling costmap window: forbid deviating there, but a
// centreline miss just bounds the active range.
constexpr double kOutOfBounds = -1.0;
constexpr char kVizNamespace[] = "survey_obstacle_avoidance";

// Build the operator-feedback overlay for a deviating tick (nominal line faint,
// adjusted path bright, the deviating band red, an "AVOIDING" flag at the peak).
visualization_msgs::msg::MarkerArray buildAvoidanceMarkers(
  const rclcpp::Time & stamp, const std::string & frame,
  const nav_msgs::msg::Path & nominal,
  const std::vector<marine_nav_utilities::Station> & stations,
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

  auto nominal_m = base(0, visualization_msgs::msg::Marker::LINE_STRIP, 0.4,
      0.6, 0.6, 0.6, 0.4);
  for (const auto & p : nominal.poses) {
    nominal_m.points.push_back(p.pose.position);
  }
  arr.markers.push_back(nominal_m);

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

  // Amber, not red: CAMP reserves red/green for the plan's editable/locked
  // state (waypoint.cpp darkRed/darkGreen), so the active-deviation band uses
  // an amber "caution" hue to stay distinct from an editable plan line.
  auto band_m = base(2, visualization_msgs::msg::Marker::LINE_LIST, 1.6,
      1.0, 0.6, 0.0, 0.9);
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

  auto text_m = base(3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.0,
      1.0, 0.6, 0.0, 1.0);  // amber, matching the deviation band (see above)
  text_m.scale.z = 3.0;
  text_m.text = "AVOIDING";
  text_m.pose.position = adjusted_point(peak_i);
  arr.markers.push_back(text_m);

  return arr;
}

// Single source of truth for the live operator-tunable parameters: the value
// default, the built-in fallback range [min, max], panel units, UI group, and
// help text. Both declareAvoidanceControlParams (declares them with descriptors)
// and bindAvoidanceControls (binds them to the marine_control server) iterate
// this, so a parameter is never declared and bound out of sync.
struct AvoidanceTunable
{
  const char * suffix;
  double default_value;
  double default_min;
  double default_max;
  const char * units;
  const char * group;
  const char * description;
};

constexpr AvoidanceTunable kTunables[] = {
  {"max_deviation", 6.0, 0.1, 100.0, "m", "geometry",
    "Corridor half-width and the give-up bound (m)."},
  {"along_track_spacing", 2.0, 0.1, 50.0, "m", "geometry",
    "Station spacing along the line (m)."},
  {"lateral_resolution", 0.5, 0.05, 10.0, "m", "geometry",
    "Cross-track search resolution (m)."},
  {"max_lateral_change", 1.0, 0.01, 50.0, "m", "geometry",
    "Max cross-track change between stations (m)."},
  {"anchor_behind_distance", 4.0, 0.0, 100.0, "m", "geometry",
    "Distance behind the boat to anchor the detour entry (m; 0 = at boat)."},
  {"line_following_weight", 1.0, 0.0, 1000.0, "", "weights",
    "Higher = hug the nominal line."},
  {"obstacle_avoidance_weight", 1.0, 0.0, 1000.0, "", "weights",
    "Higher = deviate more around obstacles."},
  {"smoothness_weight", 2.0, 0.0, 1000.0, "", "weights",
    "Higher = smoother detours."},
  {"chatter_damping_weight", 0.0, 0.0, 1000.0, "", "weights",
    "Higher = steadier tick-to-tick."},
  {"avoid_speed", 0.0, 0.0, 20.0, "m/s", "speed",
    "Inner speed limit while deviating (m/s; 0 = off)."},
};
}  // namespace

void declareAvoidanceControlParams(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  for (const auto & t : kTunables) {
    const std::string base = name + "." + t.suffix;

    // Startup-only bound parameter: [min, max] for the FloatingPointRange.
    // Platforms override it per deployment; a malformed value (wrong size/type,
    // non-finite, or min >= max) is rejected with a warning and the built-in
    // default range is used instead. Declared dynamic_typing so a platform may
    // write the bounds as an integer array (`[1, 10]`, the natural YAML form)
    // without a type-mismatch throw at declare; integer arrays are coerced below.
    rcl_interfaces::msg::ParameterDescriptor range_desc;
    range_desc.dynamic_typing = true;
    nav2_util::declare_parameter_if_not_declared(
      node, base + "_range",
      rclcpp::ParameterValue(std::vector<double>{t.default_min, t.default_max}), range_desc);

    std::vector<double> range;
    const auto range_value = node->get_parameter(base + "_range").get_parameter_value();
    if (range_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      range = range_value.get<std::vector<double>>();
    } else if (range_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto ints = range_value.get<std::vector<int64_t>>();
      range.assign(ints.begin(), ints.end());
    }  // any other type leaves `range` empty -> malformed fallback below

    double rmin = t.default_min;
    double rmax = t.default_max;
    if (range.size() == 2 && std::isfinite(range[0]) && std::isfinite(range[1]) &&
      range[0] < range[1])
    {
      rmin = range[0];
      rmax = range[1];
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "%s: malformed '%s_range' (need [min, max] with min < max); using "
        "default [%g, %g].", name.c_str(), t.suffix, t.default_min, t.default_max);
    }

    rcl_interfaces::msg::FloatingPointRange fp_range;
    fp_range.from_value = rmin;
    fp_range.to_value = rmax;
    fp_range.step = 0.0;  // continuous
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = t.description;
    descriptor.floating_point_range.push_back(fp_range);

    // Clamp the built-in default into the (possibly platform-narrowed) range so
    // declaring it never trips rclcpp's initial-value range validation. A
    // platform that overrides the *value* out of its own range fails loudly at
    // declare time, which is the correct signal for a self-contradictory config.
    const double initial = std::clamp(t.default_value, rmin, rmax);
    nav2_util::declare_parameter_if_not_declared(
      node, base, rclcpp::ParameterValue(initial), descriptor);
  }
}

void bindAvoidanceControls(marine_control::ControlServer & server, const std::string & name)
{
  for (const auto & t : kTunables) {
    server.bind_parameter(name + "." + t.suffix, t.units, t.group);
  }
}

void AvoidanceController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const char * suffix, const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node, name_ + "." + suffix, value);
    };
  declare("primary_controller", rclcpp::ParameterValue(std::string("")));

  // The live operator-tunable parameters are declared with FloatingPointRange
  // descriptors whose bounds are platform-customisable via `<name>.<t>_range`
  // startup params; this is also what the marine_control panel binds to.
  declareAvoidanceControlParams(node, name_);

  const std::string primary_type =
    node->get_parameter(name_ + ".primary_controller").as_string();
  if (primary_type.empty()) {
    throw std::runtime_error(
      name_ + ": required parameter '" + name_ + ".primary_controller' is unset "
      "(the inner controller plugin to wrap, e.g. "
      "marine_nav_crabbing_path_follower::CrabbingPathFollower)");
  }

  try {
    primary_ = primary_loader_.createSharedInstance(primary_type);
  } catch (const pluginlib::PluginlibException & ex) {
    throw std::runtime_error(
      name_ + ": failed to load primary_controller '" + primary_type + "': " + ex.what());
  }

  // Configure the inner controller under the SAME name so its existing params
  // (e.g. <name>.default_speed, <name>.pid.*) are unchanged by this wrapper —
  // the controller-config flip only adds <name>.primary_controller + the
  // avoidance tunables. The inner shares this wrapper's tf + costmap handle.
  RCLCPP_INFO(
    logger_, "%s: wrapping primary controller '%s'", name_.c_str(), primary_type.c_str());
  primary_->configure(parent, name_, tf_, costmap_ros_);

  viz_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    kVizNamespace, rclcpp::QoS(1));

  refreshParams();
}

void AvoidanceController::cleanup()
{
  if (primary_) {
    primary_->cleanup();
  }
  viz_pub_.reset();
  control_server_.reset();
}

void AvoidanceController::activate()
{
  if (primary_) {
    primary_->activate();
  }
  if (viz_pub_) {
    viz_pub_->on_activate();
  }

  // Expose the live tunables on the marine_control panel while the controller is
  // active (reset in deactivate/cleanup). The server attaches to the parent
  // controller_server node; per-plugin topics keep multiple controllers from
  // colliding on it. bind_parameter runs here, before any server callback can
  // dispatch: the controller_server is single-threaded (nav2 default), so
  // activate() owns the executor thread for the duration of this call, which
  // satisfies the control_server.hpp bind-before-spin contract. Composing the
  // controller_server into a multi-threaded executor would void that — don't,
  // without revisiting the binding-table locking.
  if (auto node = node_.lock()) {
    marine_control::ControlServerOptions options;
    options.device_name = "Survey Obstacle Avoidance (" + name_ + ")";
    options.state_topic = "~/control/" + name_ + "/state";
    options.change_topic = "~/control/" + name_ + "/change";
    control_server_ = std::make_shared<marine_control::ControlServer>(node.get(), options);
    bindAvoidanceControls(*control_server_, name_);
  }
}

void AvoidanceController::deactivate()
{
  if (primary_) {
    // Drop any active avoid_speed override before deactivating so a later
    // re-activate doesn't inherit a stale clamp.
    if (avoid_speed_active_) {
      primary_->setSpeedLimit(server_speed_limit_, server_speed_is_percentage_);
      avoid_speed_active_ = false;
    }
    primary_->deactivate();
  }
  if (viz_pub_) {
    viz_pub_->on_deactivate();
  }
  // Tear down the control channel so the panel stops advertising for an inactive
  // controller (and the heartbeat/change sub are gone before re-activation).
  control_server_.reset();
}

void AvoidanceController::setPlan(const nav_msgs::msg::Path & path)
{
  nominal_plan_ = path;
  prev_offsets_.clear();
  // A genuinely new goal starts clean: drop any active avoid_speed override so
  // the inner tracker isn't left clamped from a prior line's deviation.
  if (primary_ && avoid_speed_active_) {
    primary_->setSpeedLimit(server_speed_limit_, server_speed_is_percentage_);
    avoid_speed_active_ = false;
  }
  // The inner controller is (re-)given a plan every control cycle from
  // computeVelocityCommands once reshaping runs; seed it here too so a goal
  // with no obstacle still tracks immediately.
  if (primary_) {
    primary_->setPlan(path);
  }
}

void AvoidanceController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Remember the server-commanded limit so the avoid_speed override can restore
  // it. Only forward to the inner tracker when we are NOT currently overriding —
  // forwarding mid-override would clobber the avoid_speed cap; the new limit is
  // applied on the next restore (deviation clear / new goal / avoid_speed off).
  server_speed_limit_ = speed_limit;
  server_speed_is_percentage_ = percentage;
  if (primary_ && !avoid_speed_active_) {
    primary_->setSpeedLimit(speed_limit, percentage);
  }
}

void AvoidanceController::refreshParams()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  auto getd = [&](const char * suffix) {
      return node->get_parameter(name_ + "." + suffix).as_double();
    };
  params_.max_xte = getd("max_deviation");
  station_step_ = getd("along_track_spacing");
  params_.lateral_step = getd("lateral_resolution");
  params_.w_xte = getd("line_following_weight");
  params_.w_obs = getd("obstacle_avoidance_weight");
  params_.w_smooth = getd("smoothness_weight");
  params_.w_temporal = getd("chatter_damping_weight");
  params_.max_lateral_rate = getd("max_lateral_change");
  anchor_behind_distance_ = getd("anchor_behind_distance");
  avoid_speed_ = getd("avoid_speed");
}

geometry_msgs::msg::TwistStamped AvoidanceController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const nav_msgs::msg::Path reshaped = reshapeAroundObstacles(pose);

  // avoid_speed: while deviating, cap the inner controller's speed; restore the
  // server limit when the deviation clears OR avoid_speed is disabled. The
  // restore is intentionally NOT gated on avoid_speed_ > 0, so setting
  // avoid_speed to 0 mid-deviation un-clamps the inner rather than leaving it
  // stuck. was_avoiding_ is set by reshapeAroundObstacles for this tick.
  if (primary_) {
    const bool want_override = was_avoiding_ && avoid_speed_ > 0.0;
    if (want_override && !avoid_speed_active_) {
      primary_->setSpeedLimit(avoid_speed_, false);
      avoid_speed_active_ = true;
    } else if (!want_override && avoid_speed_active_) {
      primary_->setSpeedLimit(server_speed_limit_, server_speed_is_percentage_);
      avoid_speed_active_ = false;
    }
  }

  if (!primary_) {
    return geometry_msgs::msg::TwistStamped();
  }
  primary_->setPlan(reshaped);
  return primary_->computeVelocityCommands(pose, velocity, goal_checker);
}

nav_msgs::msg::Path AvoidanceController::reshapeAroundObstacles(
  const geometry_msgs::msg::PoseStamped & pose)
{
  auto clear_viz = [&]() {
    if (was_avoiding_ && viz_pub_) {
      visualization_msgs::msg::MarkerArray arr;
      visualization_msgs::msg::Marker del;
      del.header.frame_id = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : std::string("map");
      del.header.stamp = clock_->now();
      del.ns = kVizNamespace;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
      viz_pub_->publish(arr);
    }
    was_avoiding_ = false;
  };

  if (nominal_plan_.poses.size() < 2) {
    clear_viz();
    prev_offsets_.clear();
    return nominal_plan_;
  }

  refreshParams();

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  if (!costmap) {
    clear_viz();
    prev_offsets_.clear();
    return nominal_plan_;
  }

  const auto stations = marine_nav_utilities::resampleStations(nominal_plan_.poses, station_step_);
  if (stations.size() < 3) {
    clear_viz();
    prev_offsets_.clear();
    return nominal_plan_;
  }
  const std::size_t n = stations.size();

  // Resolve the path frame -> costmap frame transform (one lookup, reused).
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  const std::string path_frame = nominal_plan_.header.frame_id.empty()
    ? nominal_plan_.poses.front().header.frame_id
    : nominal_plan_.header.frame_id;
  tf2::Transform to_costmap;
  if (path_frame.empty() || path_frame == costmap_frame) {
    to_costmap.setIdentity();
  } else {
    try {
      const auto tf_msg = tf_->lookupTransform(costmap_frame, path_frame, tf2::TimePointZero);
      tf2::fromMsg(tf_msg.transform, to_costmap);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "%s: TF %s -> %s unavailable (%s); passing nominal path.",
        name_.c_str(), path_frame.c_str(), costmap_frame.c_str(), ex.what());
      clear_viz();
      prev_offsets_.clear();
      return nominal_plan_;
    }
  }

  // Robot position in the path frame (pose arrives in the costmap frame).
  const tf2::Vector3 robot_path =
    to_costmap.inverse() * tf2::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0);

  // Sample the costmap at a path-frame point under the costmap lock; the pure
  // planner (planCorridorOffsets) does the nearest-station + anchor-behind +
  // cost-matrix + solve. A negative return means out-of-window / impassable.
  std::optional<std::vector<double>> solved;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    auto sample = [&](double px, double py) -> double {
      const tf2::Vector3 w = to_costmap * tf2::Vector3(px, py, 0.0);
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(w.x(), w.y(), mx, my)) {
        return kOutOfBounds;
      }
      const unsigned char c = costmap->getCost(mx, my);
      if (c == nav2_costmap_2d::NO_INFORMATION) {
        return 0.0;  // unobserved == open water we simply haven't swept
      }
      return static_cast<double>(c);
    };
    solved = marine_nav_utilities::planCorridorOffsets(
      stations, sample, params_, station_step_, anchor_behind_distance_,
      robot_path.x(), robot_path.y(), prev_offsets_);
  }

  if (!solved) {
    // Out of window, no interior to deviate, or corridor blocked -> nominal.
    clear_viz();
    prev_offsets_.clear();
    return nominal_plan_;
  }

  double peak = 0.0;
  for (const double d : *solved) {
    peak = std::max(peak, std::abs(d));
  }
  prev_offsets_ = *solved;
  if (peak < kDeviationEpsilon) {
    clear_viz();
    return nominal_plan_;
  }

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

  if (viz_pub_) {
    viz_pub_->publish(
      buildAvoidanceMarkers(clock_->now(), path_frame, nominal_plan_, stations, *solved));
    was_avoiding_ = true;
  }
  return out;
}

}  // namespace marine_nav_avoidance_controller

PLUGINLIB_EXPORT_CLASS(
  marine_nav_avoidance_controller::AvoidanceController, nav2_core::Controller)
