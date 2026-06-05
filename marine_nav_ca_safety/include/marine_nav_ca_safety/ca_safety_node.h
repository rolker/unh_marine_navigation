#ifndef MARINE_NAV_CA_SAFETY_CA_SAFETY_NODE_H
#define MARINE_NAV_CA_SAFETY_CA_SAFETY_NODE_H

#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "marine_nav_ca_safety/ca_safety.h"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace marine_nav_ca_safety
{

/// Collision-avoidance **safety brake** that replaces the nav2 Collision Monitor.
/// It is intentionally *not* a maneuvering/escape node: it modulates the incoming
/// velocity command to (a) slow down — speed-scaled and yaw-preserving — as an
/// obstacle enters a forward corridor, and (b) brake to a stop with reverse thrust
/// in a close stop box, while republishing the CM's CAMP visualization (zone
/// polygons + CollisionMonitorState for the frame-vs-fill).
///
/// The node is **platform-agnostic**: all topic names are relative and frame names
/// come from parameters with generic, unprefixed defaults. The deployment applies
/// the namespace and the (roll/pitch-stabilized) frame via launch/config.
///
/// Callbacks are serialized by the default single-threaded executor (see main());
/// shared state is therefore accessed without additional locking, while live
/// parameters are kept in atomics for lock-free snapshotting.
class CaSafetyNode : public rclcpp::Node
{
public:
  explicit CaSafetyNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ca_safety", options)
  {
    // --- Construction-time parameters (read once; they wire topics/frames) ---
    cmd_vel_in_topic_ = declare_parameter<std::string>("cmd_vel_in_topic", "cmd_vel_smoothed");
    cmd_vel_out_topic_ =
      declare_parameter<std::string>("cmd_vel_out_topic", "piloting_mode/autonomous/cmd_vel");
    pointcloud_topic_ =
      declare_parameter<std::string>("pointcloud_topic", "collision_monitor/pointcloud");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    state_topic_ = declare_parameter<std::string>("state_topic", "collision_monitor_state");
    slowdown_polygon_topic_ = declare_parameter<std::string>(
      "slowdown_polygon_topic", "collision_monitor/slowdown_polygon");
    stop_polygon_topic_ =
      declare_parameter<std::string>("stop_polygon_topic", "collision_monitor/stop_polygon");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    stop_speed_eps_ = declare_parameter<double>("stop_speed_eps", 0.05);
    const double viz_rate = declare_parameter<double>("viz_rate", 2.0);
    source_loss_behavior_ = parseSourceLoss(
      declare_parameter<std::string>("source_loss_behavior", "passthrough"));

    // --- Live (dynamic) parameters: geometry/reverse tunables + toggles ---
    ttc_time_constant_ = declareDynamicDouble(
      "ttc_time_constant", 4.0, "Slowdown leading-edge time constant T (s): length = speed*T + min.");
    slowdown_min_length_ =
      declareDynamicDouble("slowdown_min_length", 5.0, "Minimum slowdown corridor length (m).");
    slowdown_max_length_ =
      declareDynamicDouble("slowdown_max_length", 25.0, "Maximum slowdown corridor length (m).");
    slowdown_speed_floor_ = declareDynamicDouble(
      "slowdown_speed_floor", 0.1, "Forward speed kept during slowdown (m/s) to preserve yaw.");
    slowdown_width_ =
      declareDynamicDouble("slowdown_width", 6.0, "Full lateral width of the slowdown corridor (m).");
    stop_length_ = declareDynamicDouble("stop_length", 5.0, "Stop box forward length (m).");
    stop_width_ = declareDynamicDouble("stop_width", 4.0, "Stop box lateral width (m).");
    reverse_speed_ =
      declareDynamicDouble("reverse_speed", 0.5, "Reverse-brake setpoint magnitude (m/s).");
    reverse_distance_ = declareDynamicDouble(
      "reverse_distance", 3.0, "Hard backstop: max reverse distance (m), independent of odom.");
    reverse_duration_ = declareDynamicDouble(
      "reverse_duration", 4.0, "Hard backstop: max reverse duration (s), independent of odom.");
    source_timeout_ =
      declareDynamicDouble("source_timeout", 1.0, "Max age of cloud/odom before considered lost (s).");
    {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description =
        "Cancel commanded yaw while reverse-braking (true = straight brake). The "
        "pass-through path (false) requires verified reverse-yaw sign handling.";
      cancel_yaw_during_reverse_ = declare_parameter<bool>("cancel_yaw_during_reverse", true, d);
    }
    {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = "Publish the CAMP zone polygons + CollisionMonitorState.";
      publish_visualization_ = declare_parameter<bool>("publish_visualization", true, d);
    }

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&CaSafetyNode::onSetParameters, this, std::placeholders::_1));
    post_param_callback_handle_ = add_post_set_parameters_callback(
      std::bind(&CaSafetyNode::onPostSetParameters, this, std::placeholders::_1));

    // --- TF ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- I/O with EXPLICIT QoS (this stack has silently lost data to implicit
    // QoS mismatches, cf. costmap #56). Pointcloud uses best-effort sensor-data
    // QoS to match nav2 sensor sources; cmd_vel/odom/viz are reliable depth-1
    // (cmd_vel matches velocity_smoother / echo_helm; viz matches CAMP's QoS(1)). ---
    cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_out_topic_, rclcpp::QoS(1));
    slowdown_poly_pub_ =
      create_publisher<geometry_msgs::msg::PolygonStamped>(slowdown_polygon_topic_, rclcpp::QoS(1));
    stop_poly_pub_ =
      create_publisher<geometry_msgs::msg::PolygonStamped>(stop_polygon_topic_, rclcpp::QoS(1));
    state_pub_ =
      create_publisher<nav2_msgs::msg::CollisionMonitorState>(state_topic_, rclcpp::QoS(1));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_in_topic_, rclcpp::QoS(1),
      std::bind(&CaSafetyNode::cmdVelCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CaSafetyNode::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(5),
      std::bind(&CaSafetyNode::odomCallback, this, std::placeholders::_1));

    // Viz + state on an independent timer so the CAMP fill (driven by
    // CollisionMonitorState, 2 s watchdog) does not blank when cmd_vel pauses.
    const double rate = isFinitePositive(viz_rate) ? viz_rate : 2.0;
    viz_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&CaSafetyNode::vizTimerCallback, this));
  }

private:
  enum class SourceLoss
  {
    Passthrough,
    Hold,
    Stop
  };

  static SourceLoss parseSourceLoss(const std::string & s)
  {
    if (s == "hold") {return SourceLoss::Hold;}
    if (s == "stop") {return SourceLoss::Stop;}
    return SourceLoss::Passthrough;
  }

  static const std::unordered_set<std::string> & dynamicDoubles()
  {
    static const std::unordered_set<std::string> names{
      "ttc_time_constant", "slowdown_min_length", "slowdown_max_length", "slowdown_speed_floor",
      "slowdown_width", "stop_length", "stop_width", "reverse_speed", "reverse_distance",
      "reverse_duration", "source_timeout"};
    return names;
  }

  double declareDynamicDouble(const std::string & name, double def, const std::string & desc)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.dynamic_typing = true;  // accept a bare integer and coerce in the callback
    d.description = desc + " Finite and > 0.";
    return declare_parameter(name, def, d);
  }

  static bool numericFromParameter(const rclcpp::Parameter & p, double & value)
  {
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        value = p.as_double();
        return true;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        value = static_cast<double>(p.as_int());
        return true;
      default:
        return false;
    }
  }

  // Pre-set: validate only, no side effects (a SetParameters request is atomic).
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : parameters) {
      if (dynamicDoubles().count(p.get_name()) == 0) {
        continue;
      }
      double v;
      if (!numericFromParameter(p, v)) {
        result.successful = false;
        result.reason = p.get_name() + " must be a number (double or integer)";
        break;
      }
      if (!isFinitePositive(v)) {
        result.successful = false;
        result.reason = p.get_name() + " must be finite and > 0";
        break;
      }
    }
    return result;
  }

  // Post-set: apply validated values (fires only after the request commits).
  void onPostSetParameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & p : parameters) {
      const std::string & n = p.get_name();
      double v;
      if (dynamicDoubles().count(n) && numericFromParameter(p, v)) {
        if (n == "ttc_time_constant") {ttc_time_constant_ = v;} else if (n == "slowdown_min_length") {
          slowdown_min_length_ = v;
        } else if (n == "slowdown_max_length") {slowdown_max_length_ = v;} else if (
          n == "slowdown_speed_floor")
        {
          slowdown_speed_floor_ = v;
        } else if (n == "slowdown_width") {slowdown_width_ = v;} else if (n == "stop_length") {
          stop_length_ = v;
        } else if (n == "stop_width") {stop_width_ = v;} else if (n == "reverse_speed") {
          reverse_speed_ = v;
        } else if (n == "reverse_distance") {reverse_distance_ = v;} else if (
          n == "reverse_duration")
        {
          reverse_duration_ = v;
        } else if (n == "source_timeout") {source_timeout_ = v;}
      } else if (n == "cancel_yaw_during_reverse" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        cancel_yaw_during_reverse_ = p.as_bool();
      } else if (n == "publish_visualization" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        publish_visualization_ = p.as_bool();
      }
    }
  }

  SafetyParams snapshotParams() const
  {
    SafetyParams p;
    p.ttc_time_constant = ttc_time_constant_.load();
    p.slowdown_min_length = slowdown_min_length_.load();
    p.slowdown_max_length = slowdown_max_length_.load();
    p.slowdown_speed_floor = slowdown_speed_floor_.load();
    p.slowdown_width = slowdown_width_.load();
    p.stop_length = stop_length_.load();
    p.stop_width = stop_width_.load();
    p.reverse_speed = reverse_speed_.load();
    p.cancel_yaw_during_reverse = cancel_yaw_during_reverse_.load();
    return p;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2 & msg)
  {
    last_cloud_ = msg;
    have_cloud_ = true;
    last_cloud_time_ = now();
  }

  void odomCallback(const nav_msgs::msg::Odometry & msg)
  {
    measured_speed_ = msg.twist.twist.linear.x;
    odom_x_ = msg.pose.pose.position.x;
    odom_y_ = msg.pose.pose.position.y;
    have_odom_ = true;
    last_odom_time_ = now();
  }

  // Transform the cached cloud into base_frame and extract (x, y) points.
  bool transformCloud(std::vector<Point2> & pts)
  {
    if (!have_cloud_) {
      return false;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, last_cloud_.header.frame_id, tf2::TimePointZero);
      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(last_cloud_, transformed, tf);
      pts.clear();
      sensor_msgs::PointCloud2ConstIterator<float> ix(transformed, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(transformed, "y");
      for (; ix != ix.end(); ++ix, ++iy) {
        pts.push_back(Point2{static_cast<double>(*ix), static_cast<double>(*iy)});
      }
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CA safety: cloud transform to %s failed: %s",
        base_frame_.c_str(), e.what());
      return false;
    }
  }

  void resetReverse() {reverse_active_ = false;}

  // Reverse-brake with a hard distance/duration backstop that does NOT depend on
  // odom, so an odom dropout mid-reverse cannot cause aft runaway.
  Twist2 doStop(const Twist2 & in, double speed, bool odom_fresh, const SafetyParams & p)
  {
    const rclcpp::Time t = now();
    if (!reverse_active_) {
      reverse_active_ = true;
      reverse_start_time_ = t;
      reverse_start_has_pose_ = odom_fresh;
      reverse_start_x_ = odom_x_;
      reverse_start_y_ = odom_y_;
    }
    const bool within_duration = (t - reverse_start_time_).seconds() < reverse_duration_.load();
    bool within_distance = true;
    if (odom_fresh && reverse_start_has_pose_) {
      within_distance =
        std::hypot(odom_x_ - reverse_start_x_, odom_y_ - reverse_start_y_) < reverse_distance_.load();
    }
    const bool reverse_allowed = within_duration && within_distance;
    // With odom stale we cannot tell we've stopped; keep braking until the
    // duration backstop ends it (treat as still moving).
    const double speed_for_stop = odom_fresh ? speed : std::numeric_limits<double>::infinity();
    return applyStop(in, speed_for_stop, reverse_allowed, p, stop_speed_eps_);
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped & msg)
  {
    const SafetyParams p = snapshotParams();
    const rclcpp::Time t = now();
    const double timeout = source_timeout_.load();

    const bool odom_fresh = have_odom_ && (t - last_odom_time_).seconds() <= timeout;
    const double speed = odom_fresh ? measured_speed_ : 0.0;

    const bool cloud_fresh = have_cloud_ && (t - last_cloud_time_).seconds() <= timeout;
    const bool use_cloud = cloud_fresh || (have_cloud_ && source_loss_behavior_ == SourceLoss::Hold);

    std::vector<Point2> pts;
    const bool cloud_usable = use_cloud && transformCloud(pts);

    const Twist2 in{msg.twist.linear.x, msg.twist.angular.z};
    Twist2 out = in;
    Zone zone = Zone::Clear;

    if (cloud_usable) {
      const double slow_len = slowdownLength(speed, p);
      const double slow_range = nearestForwardRange(pts, p.slowdown_width, slow_len);
      const double stop_range = nearestForwardRange(pts, p.stop_width, p.stop_length);
      zone = classify(stop_range, slow_range, slow_len);
      switch (zone) {
        case Zone::Clear:
          out = in;
          resetReverse();
          break;
        case Zone::Slowdown:
          out = applySlowdown(
            in, slowdownScale(slow_range, slow_len, p.stop_length), p.slowdown_speed_floor);
          resetReverse();
          break;
        case Zone::Stop:
          out = doStop(in, speed, odom_fresh, p);
          break;
      }
    } else if (source_loss_behavior_ == SourceLoss::Stop && have_cloud_) {
      // Source lost (and not holding): conservative brake.
      zone = Zone::Stop;
      out = doStop(in, speed, odom_fresh, p);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CA safety: obstacle source lost — braking (stop policy).");
    } else {
      // Passthrough (default) or no cloud ever received.
      out = in;
      resetReverse();
      if (have_cloud_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "CA safety: obstacle source lost — passing commands through.");
      }
    }

    current_zone_ = zone;

    geometry_msgs::msg::TwistStamped out_msg = msg;  // preserve other twist components + header
    out_msg.header.stamp = t;
    out_msg.twist.linear.x = out.linear_x;
    out_msg.twist.angular.z = out.angular_z;
    cmd_vel_pub_->publish(out_msg);
  }

  geometry_msgs::msg::PolygonStamped makePolygon(double length, double width, const rclcpp::Time & t)
  {
    geometry_msgs::msg::PolygonStamped poly;
    poly.header.frame_id = base_frame_;
    poly.header.stamp = t;
    for (const auto & c : forwardBoxCorners(length, width)) {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(c.x);
      pt.y = static_cast<float>(c.y);
      pt.z = 0.0f;
      poly.polygon.points.push_back(pt);
    }
    return poly;
  }

  void vizTimerCallback()
  {
    if (!publish_visualization_.load()) {
      return;
    }
    const rclcpp::Time t = now();
    const SafetyParams p = snapshotParams();
    const bool odom_fresh = have_odom_ && (t - last_odom_time_).seconds() <= source_timeout_.load();
    const double speed = odom_fresh ? measured_speed_ : 0.0;

    slowdown_poly_pub_->publish(makePolygon(slowdownLength(speed, p), p.slowdown_width, t));
    stop_poly_pub_->publish(makePolygon(p.stop_length, p.stop_width, t));

    nav2_msgs::msg::CollisionMonitorState state;
    switch (current_zone_) {
      case Zone::Stop:
        state.action_type = nav2_msgs::msg::CollisionMonitorState::STOP;
        break;
      case Zone::Slowdown:
        state.action_type = nav2_msgs::msg::CollisionMonitorState::SLOWDOWN;
        break;
      case Zone::Clear:
        state.action_type = nav2_msgs::msg::CollisionMonitorState::DO_NOTHING;
        break;
    }
    state_pub_->publish(state);
  }

  // Construction-time config.
  std::string cmd_vel_in_topic_, cmd_vel_out_topic_, pointcloud_topic_, odom_topic_, state_topic_;
  std::string slowdown_polygon_topic_, stop_polygon_topic_, base_frame_;
  double stop_speed_eps_{0.05};
  SourceLoss source_loss_behavior_{SourceLoss::Passthrough};

  // Live params.
  std::atomic<double> ttc_time_constant_{4.0}, slowdown_min_length_{5.0}, slowdown_max_length_{25.0};
  std::atomic<double> slowdown_speed_floor_{0.1}, slowdown_width_{6.0}, stop_length_{5.0};
  std::atomic<double> stop_width_{4.0}, reverse_speed_{0.5}, reverse_distance_{3.0};
  std::atomic<double> reverse_duration_{4.0}, source_timeout_{1.0};
  std::atomic<bool> cancel_yaw_during_reverse_{true}, publish_visualization_{true};

  // Cached inputs (single-threaded executor; no extra locking).
  sensor_msgs::msg::PointCloud2 last_cloud_;
  bool have_cloud_{false};
  rclcpp::Time last_cloud_time_;
  double measured_speed_{0.0}, odom_x_{0.0}, odom_y_{0.0};
  bool have_odom_{false};
  rclcpp::Time last_odom_time_;

  // Reverse-brake backstop state.
  bool reverse_active_{false};
  bool reverse_start_has_pose_{false};
  rclcpp::Time reverse_start_time_;
  double reverse_start_x_{0.0}, reverse_start_y_{0.0};

  Zone current_zone_{Zone::Clear};

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  PostSetParametersCallbackHandle::SharedPtr post_param_callback_handle_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr slowdown_poly_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr stop_poly_pub_;
  rclcpp::Publisher<nav2_msgs::msg::CollisionMonitorState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr viz_timer_;
};

}  // namespace marine_nav_ca_safety

#endif  // MARINE_NAV_CA_SAFETY_CA_SAFETY_NODE_H
