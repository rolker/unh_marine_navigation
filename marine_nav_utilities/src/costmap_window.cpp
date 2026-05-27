#include "marine_nav_utilities/costmap_window.h"

#include <algorithm>
#include <cmath>

#include "tf2/utils.h"

namespace marine_nav_utilities
{

nav_msgs::msg::OccupancyGrid cropCostmapWindow(
  const nav_msgs::msg::OccupancyGrid & input,
  double window_size_meters)
{
  const unsigned int width = input.info.width;
  const unsigned int height = input.info.height;
  const double resolution = input.info.resolution;

  // Nothing meaningful to crop from a degenerate grid or a bad window size.
  // Reject a non-finite resolution explicitly: a NaN slips past a bare `<= 0.0`
  // test and std::clamp with a NaN bound is undefined behavior.
  if (!std::isfinite(resolution) || resolution <= 0.0 || width == 0 || height == 0 ||
    !windowSizeIsValid(window_size_meters))
  {
    return input;
  }

  // A well-formed OccupancyGrid has exactly width*height cells; bail rather than
  // read out of bounds on a malformed message.
  if (input.data.size() != static_cast<size_t>(width) * height) {
    return input;
  }

  // Window side in cells, clamped to fit within the grid.
  const unsigned int min_dim = std::min(width, height);
  const double requested_cells = std::round(window_size_meters / resolution);
  const unsigned int side = static_cast<unsigned int>(
    std::clamp(requested_cells, 1.0, static_cast<double>(min_dim)));

  // Center-crop offsets. Integer division puts any odd leftover cell on the high
  // side, which is the conventional centered-window behavior.
  const unsigned int x0 = (width - side) / 2;
  const unsigned int y0 = (height - side) / 2;

  nav_msgs::msg::OccupancyGrid output;
  output.header = input.header;
  // Copy all metadata (resolution, map_load_time, origin, ...), then override
  // the dimensions and origin below. Copying wholesale keeps any future
  // MapMetaData field carried over instead of silently dropped.
  output.info = input.info;
  output.info.width = side;
  output.info.height = side;

  // Shift the origin to the cropped corner. info.origin is the pose of cell
  // (0,0); the new corner is (x0, y0) cells away along the grid's local axes, so
  // rotate that offset by the origin orientation before adding it to the origin
  // position. Orientation is unchanged. A zero/unset quaternion is treated as
  // identity (no rotation).
  tf2::Quaternion q(
    input.info.origin.orientation.x, input.info.origin.orientation.y,
    input.info.origin.orientation.z, input.info.origin.orientation.w);
  if (!std::isfinite(q.length2()) || q.length2() < 1e-9) {
    q = tf2::Quaternion::getIdentity();
  }
  const tf2::Vector3 world_offset =
    tf2::Matrix3x3(q.normalized()) *
    tf2::Vector3(x0 * resolution, y0 * resolution, 0.0);

  output.info.origin.position.x += world_offset.x();
  output.info.origin.position.y += world_offset.y();
  output.info.origin.position.z += world_offset.z();

  // Copy the windowed cells row by row (row-major, row 0 at the origin).
  output.data.resize(static_cast<size_t>(side) * side);
  for (unsigned int row = 0; row < side; ++row) {
    const auto in_begin =
      input.data.begin() + static_cast<size_t>(y0 + row) * width + x0;
    std::copy(
      in_begin, in_begin + side,
      output.data.begin() + static_cast<size_t>(row) * side);
  }

  return output;
}

}  // namespace marine_nav_utilities
