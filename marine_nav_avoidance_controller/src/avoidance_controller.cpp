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

  auto text_m = base(3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.0,
      1.0, 0.2, 0.15, 1.0);
  text_m.scale.z = 3.0;
  text_m.text = "AVOIDING";
  text_m.pose.position = adjusted_point(peak_i);
  arr.markers.push_back(text_m);

  return arr;
}
}  // namespace

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

  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node, name_ + ".primary_controller", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(node, name_ + ".max_deviation", rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(node, name_ + ".along_track_spacing", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name_ + ".lateral_resolution", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".line_following_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".obstacle_avoidance_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".smoothness_weight", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name_ + ".chatter_damping_weight", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, name_ + ".max_lateral_change", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".anchor_behind_distance", rclcpp::ParameterValue(4.0));
  declare_parameter_if_not_declared(node, name_ + ".avoid_speed", rclcpp::ParameterValue(0.0));

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
}

void AvoidanceController::activate()
{
  if (primary_) {
    primary_->activate();
  }
  if (viz_pub_) {
    viz_pub_->on_activate();
  }
}

void AvoidanceController::deactivate()
{
  if (primary_) {
    primary_->deactivate();
  }
  if (viz_pub_) {
    viz_pub_->on_deactivate();
  }
}

void AvoidanceController::setPlan(const nav_msgs::msg::Path & path)
{
  nominal_plan_ = path;
  prev_offsets_.clear();
  // The inner controller is (re-)given a plan every control cycle from
  // computeVelocityCommands once reshaping runs; seed it here too so a goal
  // with no obstacle still tracks immediately.
  if (primary_) {
    primary_->setPlan(path);
  }
}

void AvoidanceController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Remember the server-commanded limit so we can restore it after an
  // avoid_speed override, and forward it to the inner tracker.
  server_speed_limit_ = speed_limit;
  server_speed_is_percentage_ = percentage;
  avoid_speed_active_ = false;
  if (primary_) {
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
  // server-commanded limit on the deviating->clear transition. was_avoiding_ is
  // set by reshapeAroundObstacles for this tick.
  if (primary_ && avoid_speed_ > 0.0) {
    if (was_avoiding_ && !avoid_speed_active_) {
      primary_->setSpeedLimit(avoid_speed_, false);
      avoid_speed_active_ = true;
    } else if (!was_avoiding_ && avoid_speed_active_) {
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
