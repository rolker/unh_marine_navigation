#include <functional>
#include <memory>

#include "marine_nav_utilities/costmap_window.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace marine_nav_utilities
{

/// Republishes a cropped, smaller window of an OccupancyGrid (typically a Nav2
/// rolling local costmap) so it is cheap enough to forward to the operator over
/// a lossy, rate-limited link. The output is a full, self-contained grid each
/// time, so it tolerates dropped frames without any delta-stitching downstream.
class CostmapWindowNode : public rclcpp::Node
{
public:
  explicit CostmapWindowNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("costmap_window", options)
  {
    window_size_ = declare_parameter("window_size", 200.0);

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
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

}  // namespace marine_nav_utilities

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<marine_nav_utilities::CostmapWindowNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
