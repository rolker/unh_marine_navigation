#ifndef MARINE_NAV_UTILITIES_COSTMAP_WINDOW_NODE_H
#define MARINE_NAV_UTILITIES_COSTMAP_WINDOW_NODE_H

#include <functional>
#include <vector>

#include "marine_nav_utilities/costmap_window.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
    window_size_ = declare_parameter("window_size", 200.0);

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&CostmapWindowNode::onSetParameters, this, std::placeholders::_1));

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
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "window_size") {
        const double value = parameter.as_double();
        if (!windowSizeIsValid(value)) {
          result.successful = false;
          result.reason = "window_size must be finite and > 0";
        } else {
          window_size_ = value;
        }
      }
    }
    return result;
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid & grid)
  {
    const auto windowed = cropCostmapWindow(grid, window_size_);
    if (!logged_) {
      RCLCPP_INFO(
        get_logger(),
        "Cropping costmap %ux%u @ %.3f m -> %ux%u (window %.1f m)",
        grid.info.width, grid.info.height, grid.info.resolution,
        windowed.info.width, windowed.info.height, window_size_);
      logged_ = true;
    }
    publisher_->publish(windowed);
  }

  double window_size_{200.0};
  bool logged_{false};
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

}  // namespace marine_nav_utilities

#endif  // MARINE_NAV_UTILITIES_COSTMAP_WINDOW_NODE_H
