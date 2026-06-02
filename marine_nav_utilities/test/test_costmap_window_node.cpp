#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

// #56: the "costmap" subscription must explicitly advertise reliable +
// transient-local QoS so it deterministically matches (and counts toward the
// subscription-count gate of) Nav2's reliable+transient-local costmap publisher
// — `Costmap2DPublisher::publishCostmap()` only emits when
// `get_subscription_count() > 0`. The previous `best_available` policies made the
// match nondeterministic at startup, which is why a manual second subscriber was
// needed. Guard against a regression back to best_available / volatile.
TEST_F(CostmapWindowNodeTest, CostmapSubscriptionIsReliableTransientLocal)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  std::vector<rclcpp::TopicEndpointInfo> infos;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    infos = node_->get_subscriptions_info_by_topic("/costmap");
    if (!infos.empty()) {
      break;
    }
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_FALSE(infos.empty()) << "no subscription discovered on /costmap";
  bool found_reliable_transient_local = false;
  for (const auto & info : infos) {
    if (info.qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable &&
      info.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal)
    {
      found_reliable_transient_local = true;
    }
  }
  EXPECT_TRUE(found_reliable_transient_local)
    << "costmap subscription must be reliable + transient_local (matches Nav2; #56)";
}
