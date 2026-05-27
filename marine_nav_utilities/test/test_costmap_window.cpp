#include <cmath>

#include <gtest/gtest.h>

#include "marine_nav_utilities/costmap_window.h"

namespace
{

// Build a grid with identity origin orientation and data[i] = i % 100, so every
// cell is individually identifiable for crop-correctness checks.
nav_msgs::msg::OccupancyGrid makeGrid(
  unsigned int width, unsigned int height, double resolution,
  double origin_x = 0.0, double origin_y = 0.0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(static_cast<size_t>(width) * height);
  for (size_t i = 0; i < grid.data.size(); ++i) {
    grid.data[i] = static_cast<int8_t>(i % 100);
  }
  return grid;
}

}  // namespace

using marine_nav_utilities::cropCostmapWindow;

TEST(CropCostmapWindow, BasicCenterCrop)
{
  const auto in = makeGrid(8, 8, 1.0);
  const auto out = cropCostmapWindow(in, 4.0);

  EXPECT_EQ(out.info.width, 4u);
  EXPECT_EQ(out.info.height, 4u);
  EXPECT_DOUBLE_EQ(out.info.resolution, 1.0);
  // x0 = y0 = (8 - 4) / 2 = 2 cells -> origin shifts by 2 m.
  EXPECT_DOUBLE_EQ(out.info.origin.position.x, 2.0);
  EXPECT_DOUBLE_EQ(out.info.origin.position.y, 2.0);
  ASSERT_EQ(out.data.size(), 16u);
  for (unsigned int row = 0; row < 4; ++row) {
    for (unsigned int col = 0; col < 4; ++col) {
      EXPECT_EQ(out.data[row * 4 + col], in.data[(row + 2) * 8 + (col + 2)])
        << "row=" << row << " col=" << col;
    }
  }
}

TEST(CropCostmapWindow, ResolutionScalesOriginShift)
{
  const auto in = makeGrid(8, 8, 0.5, /*origin_x=*/-10.0, /*origin_y=*/5.0);
  const auto out = cropCostmapWindow(in, 2.0);  // 2 m / 0.5 = 4 cells

  EXPECT_EQ(out.info.width, 4u);
  EXPECT_EQ(out.info.height, 4u);
  // x0 = (8 - 4) / 2 = 2 cells -> 2 * 0.5 = 1.0 m shift from the existing origin.
  EXPECT_DOUBLE_EQ(out.info.origin.position.x, -10.0 + 1.0);
  EXPECT_DOUBLE_EQ(out.info.origin.position.y, 5.0 + 1.0);
}

TEST(CropCostmapWindow, WindowLargerThanGridClampsToGrid)
{
  const auto in = makeGrid(8, 8, 1.0);
  const auto out = cropCostmapWindow(in, 100.0);

  EXPECT_EQ(out.info.width, 8u);
  EXPECT_EQ(out.info.height, 8u);
  EXPECT_DOUBLE_EQ(out.info.origin.position.x, 0.0);
  EXPECT_DOUBLE_EQ(out.info.origin.position.y, 0.0);
  EXPECT_EQ(out.data, in.data);
}

TEST(CropCostmapWindow, NonSquareGridClampsToShorterSide)
{
  const auto in = makeGrid(10, 6, 1.0);
  const auto out = cropCostmapWindow(in, 4.0);

  EXPECT_EQ(out.info.width, 4u);
  EXPECT_EQ(out.info.height, 4u);
  // x0 = (10 - 4) / 2 = 3, y0 = (6 - 4) / 2 = 1.
  EXPECT_DOUBLE_EQ(out.info.origin.position.x, 3.0);
  EXPECT_DOUBLE_EQ(out.info.origin.position.y, 1.0);
  for (unsigned int row = 0; row < 4; ++row) {
    for (unsigned int col = 0; col < 4; ++col) {
      EXPECT_EQ(out.data[row * 4 + col], in.data[(row + 1) * 10 + (col + 3)]);
    }
  }
}

TEST(CropCostmapWindow, RotatedOriginShiftsAlongGridAxes)
{
  auto in = makeGrid(8, 8, 1.0);
  // Rotate origin +90 deg about z: the grid's local +x points along world +y.
  in.info.origin.orientation.z = std::sin(M_PI / 4.0);
  in.info.origin.orientation.w = std::cos(M_PI / 4.0);

  const auto out = cropCostmapWindow(in, 4.0);  // x0 = y0 = 2 cells

  // Local corner offset (2, 2) rotated +90 deg -> world (-2, 2).
  EXPECT_NEAR(out.info.origin.position.x, -2.0, 1e-9);
  EXPECT_NEAR(out.info.origin.position.y, 2.0, 1e-9);
}

TEST(CropCostmapWindow, DegenerateGridReturnedUnchanged)
{
  const auto zero_res = makeGrid(8, 8, 0.0);
  const auto out = cropCostmapWindow(zero_res, 4.0);
  EXPECT_EQ(out.info.width, 8u);
  EXPECT_EQ(out.info.height, 8u);
  EXPECT_EQ(out.data, zero_res.data);

  nav_msgs::msg::OccupancyGrid empty;
  empty.info.resolution = 1.0;  // width / height left at zero
  const auto out_empty = cropCostmapWindow(empty, 4.0);
  EXPECT_EQ(out_empty.info.width, 0u);
  EXPECT_EQ(out_empty.info.height, 0u);
}

TEST(CropCostmapWindow, InvalidWindowSizeReturnedUnchanged)
{
  const auto in = makeGrid(8, 8, 1.0);
  // A non-finite or non-positive window must not produce a malformed (0x0)
  // grid; the input is returned unchanged.
  for (const double bad : {std::nan(""), 0.0, -4.0}) {
    const auto out = cropCostmapWindow(in, bad);
    EXPECT_EQ(out.info.width, 8u) << "window=" << bad;
    EXPECT_EQ(out.info.height, 8u) << "window=" << bad;
    EXPECT_EQ(out.data, in.data) << "window=" << bad;
  }
}

TEST(CropCostmapWindow, NonFiniteResolutionReturnedUnchanged)
{
  auto nan_res = makeGrid(8, 8, 1.0);
  nan_res.info.resolution = std::nan("");  // NaN slips past a bare <= 0 test
  const auto out = cropCostmapWindow(nan_res, 4.0);
  EXPECT_EQ(out.info.width, 8u);
  EXPECT_EQ(out.info.height, 8u);
  EXPECT_EQ(out.data, nan_res.data);
}

TEST(CropCostmapWindow, MismatchedDataSizeReturnedUnchanged)
{
  auto bad = makeGrid(8, 8, 1.0);
  bad.data.resize(10);  // fewer cells than width * height -> would read OOB
  const auto out = cropCostmapWindow(bad, 4.0);
  EXPECT_EQ(out.info.width, 8u);
  EXPECT_EQ(out.info.height, 8u);
  EXPECT_EQ(out.data.size(), 10u);
}

TEST(CropCostmapWindow, PreservesMetadata)
{
  auto in = makeGrid(8, 8, 1.0);
  in.header.frame_id = "map_tide";
  in.info.map_load_time.sec = 123;
  in.info.map_load_time.nanosec = 456;

  const auto out = cropCostmapWindow(in, 4.0);

  EXPECT_EQ(out.header.frame_id, "map_tide");
  EXPECT_EQ(out.info.map_load_time.sec, 123);
  EXPECT_EQ(out.info.map_load_time.nanosec, 456u);
}
