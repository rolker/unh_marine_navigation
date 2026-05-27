#ifndef MARINE_NAV_UTILITIES_COSTMAP_WINDOW_NODE_H
#define MARINE_NAV_UTILITIES_COSTMAP_WINDOW_NODE_H

#include <atomic>
#include <functional>
#include <vector>

#include "marine_nav_utilities/costmap_window.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace marine_nav_utilities
{

/// Republishes a cropped, smaller window of an OccupancyGrid (typically a Nav2
/// rolling local costmap) so it is cheap enough to forward to the operator over
/// a lossy, rate-limited link. The output is a full, self-contained grid each
/// time, so it tolerates dropped frames without any delta-stitching downstream.
///
/// The `window_size` parameter (meters) is dynamically updatable: a set-parameter
/// request with a non-finite or non-positive value is rejected so the operator
/// gets immediate feedback rather than a silent no-crop.
class CostmapWindowNode : public rclcpp::Node
{
public:
  explicit CostmapWindowNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("costmap_window", options)
  {
    // Dynamic typing so a bare integer (`ros2 param set ... window_size 100`)
    // reaches the callback to be coerced, rather than being rejected up front by
    // static type checking. The callback enforces the type and value.
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    descriptor.description =
      "Side length in meters of the square window cropped from the costmap "
      "(finite, > 0). An integer value is accepted and used as a double "
      "internally; the stored parameter keeps its original type.";
    window_size_ = declare_parameter("window_size", 200.0, descriptor);

    // Validate in the pre-set callback (no side effects) and apply only in the
    // post-set callback, which fires after the whole atomic request commits.
    // This keeps window_size_ in sync with the parameter store even when a
    // co-set parameter in the same request is rejected.
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&CostmapWindowNode::onSetParameters, this, std::placeholders::_1));
    post_param_callback_handle_ = add_post_set_parameters_callback(
      std::bind(&CostmapWindowNode::onPostSetParameters, this, std::placeholders::_1));

    // Nav2 publishes the costmap reliable + transient-local (latched). Use
    // best-available for both policies so we attach and receive the streamed
    // updates regardless of how the source is configured.
    rclcpp::QoS sub_qos(1);
    sub_qos.reliability_best_available();
    sub_qos.durability_best_available();
    subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "costmap", sub_qos,
      std::bind(&CostmapWindowNode::costmapCallback, this, std::placeholders::_1));

    // Publish latched so a late-joining consumer (CAMP / the bridge) immediately
    // gets the most recent window.
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "costmap_windowed", rclcpp::QoS(1).transient_local());
  }

private:
  // Read a window_size parameter as a double, accepting INTEGER or DOUBLE.
  // Returns false (leaving \p value untouched) for any other type.
  static bool windowSizeFromParameter(const rclcpp::Parameter & parameter, double & value)
  {
    switch (parameter.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        value = parameter.as_double();
        return true;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        value = static_cast<double>(parameter.as_int());
        return true;
      default:
        return false;
    }
  }

  // Pre-set: validate only, with no side effects. A SetParameters request is
  // atomic — returning unsuccessful means none of its parameters are applied —
  // so mutating state here would risk diverging from the parameter store.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() != "window_size") {
        continue;
      }
      double value;
      if (!windowSizeFromParameter(parameter, value)) {
        result.successful = false;
        result.reason = "window_size must be a number (double or integer)";
        break;
      }
      if (!windowSizeIsValid(value)) {
        result.successful = false;
        result.reason = "window_size must be finite and > 0";
        break;
      }
    }
    return result;
  }

  // Post-set: apply the validated value, called only after the request commits.
  void onPostSetParameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & parameter : parameters) {
      double value;
      if (parameter.get_name() == "window_size" &&
        windowSizeFromParameter(parameter, value))
      {
        window_size_ = value;
      }
    }
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid & grid)
  {
    const double window_size = window_size_.load();
    const auto windowed = cropCostmapWindow(grid, window_size);
    if (!logged_) {
      RCLCPP_INFO(
        get_logger(),
        "Cropping costmap %ux%u @ %.3f m -> %ux%u (window %.1f m)",
        grid.info.width, grid.info.height, grid.info.resolution,
        windowed.info.width, windowed.info.height, window_size);
      logged_ = true;
    }
    publisher_->publish(windowed);
  }

  std::atomic<double> window_size_{200.0};
  bool logged_{false};
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  PostSetParametersCallbackHandle::SharedPtr post_param_callback_handle_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

}  // namespace marine_nav_utilities

#endif  // MARINE_NAV_UTILITIES_COSTMAP_WINDOW_NODE_H
