#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "marine_nav_utilities/costmap_window_node.h"
#include "rclcpp/rclcpp.hpp"

using marine_nav_utilities::CostmapWindowNode;

class CostmapWindowNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<CostmapWindowNode>(rclcpp::NodeOptions());
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<CostmapWindowNode> node_;
};

TEST_F(CostmapWindowNodeTest, WindowSizeDefaultsTo200)
{
  EXPECT_DOUBLE_EQ(node_->get_parameter("window_size").as_double(), 200.0);
}

TEST_F(CostmapWindowNodeTest, ValidWindowSizeUpdatesDynamically)
{
  const auto result = node_->set_parameter(rclcpp::Parameter("window_size", 100.0));
  EXPECT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("window_size").as_double(), 100.0);
}

TEST_F(CostmapWindowNodeTest, InvalidWindowSizeRejectedAndValueUnchanged)
{
  for (const double bad : {0.0, -5.0, std::nan(""), std::numeric_limits<double>::infinity()}) {
    const auto result = node_->set_parameter(rclcpp::Parameter("window_size", bad));
    EXPECT_FALSE(result.successful) << "bad=" << bad;
    // Rejected sets must leave the previous (default) value intact.
    EXPECT_DOUBLE_EQ(node_->get_parameter("window_size").as_double(), 200.0) << "bad=" << bad;
  }
}

TEST_F(CostmapWindowNodeTest, IntegerWindowSizeAcceptedAndUsedAsDouble)
{
  // A bare integer (`ros2 param set ... window_size 150`) is accepted via dynamic
  // typing rather than rejected as a type mismatch, and used as a double
  // internally. The stored parameter keeps its integer type.
  const auto result = node_->set_parameter(rclcpp::Parameter("window_size", 150));
  EXPECT_TRUE(result.successful);
  EXPECT_EQ(node_->get_parameter("window_size").as_int(), 150);
}

TEST_F(CostmapWindowNodeTest, NonNumericWindowSizeRejected)
{
  const auto result = node_->set_parameter(rclcpp::Parameter("window_size", std::string("wide")));
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("window_size").as_double(), 200.0);
}
