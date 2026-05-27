#ifndef MARINE_NAV_UTILITIES_COSTMAP_WINDOW_H
#define MARINE_NAV_UTILITIES_COSTMAP_WINDOW_H

#include <cmath>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace marine_nav_utilities
{

/// A window size (meters) is usable only if it is finite and positive. A NaN
/// would otherwise slip past a bare `<= 0` test into std::clamp (undefined
/// behavior). Shared by cropCostmapWindow() and the node's parameter validation.
inline bool windowSizeIsValid(double window_size_meters)
{
  return std::isfinite(window_size_meters) && window_size_meters > 0.0;
}

/// Center-crop a square window out of an OccupancyGrid.
///
/// Extracts a square sub-grid `window_size_meters` on a side, centered on the
/// geometric center of \p input, at the input resolution (no resampling). The
/// result is a valid OccupancyGrid: same \c header and \c resolution, \c width /
/// \c height set to the window, and \c info.origin shifted to the cropped
/// region's corner. The shift is applied along the grid's own axes via the input
/// origin orientation, so it is correct even for a rotated grid.
///
/// For a Nav2 \c rolling_window costmap the robot sits at the grid center, so a
/// center crop is robot-centered. For a static/global grid this crops the
/// geometric center, which is not the robot.
///
/// The window side in cells is `round(window_size_meters / resolution)`, clamped
/// to `[1, min(width, height)]`. If \p input has a non-positive resolution or no
/// cells, it is returned unchanged.
nav_msgs::msg::OccupancyGrid cropCostmapWindow(
  const nav_msgs::msg::OccupancyGrid & input,
  double window_size_meters);

}  // namespace marine_nav_utilities

#endif  // MARINE_NAV_UTILITIES_COSTMAP_WINDOW_H
