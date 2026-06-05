#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "marine_nav_ca_safety/ca_safety_node.h"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "rclcpp/rclcpp.hpp"

using marine_nav_ca_safety::CaSafetyNode;
using namespace std::chrono_literals;

class CaSafetyNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<CaSafetyNode>(rclcpp::NodeOptions());
  }
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }
  std::shared_ptr<CaSafetyNode> node_;
};

TEST_F(CaSafetyNodeTest, DefaultsAreSane)
{
  EXPECT_DOUBLE_EQ(node_->get_parameter("ttc_time_constant").as_double(), 4.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("slowdown_min_length").as_double(), 5.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("reverse_speed").as_double(), 0.5);
  EXPECT_TRUE(node_->get_parameter("cancel_yaw_during_reverse").as_bool());
  EXPECT_EQ(node_->get_parameter("base_frame").as_string(), "base_link");
  EXPECT_EQ(node_->get_parameter("source_loss_behavior").as_string(), "passthrough");
}

TEST_F(CaSafetyNodeTest, ValidDynamicUpdateApplied)
{
  const auto r = node_->set_parameter(rclcpp::Parameter("ttc_time_constant", 6.0));
  EXPECT_TRUE(r.successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("ttc_time_constant").as_double(), 6.0);
}

TEST_F(CaSafetyNodeTest, InvalidNumericRejectedValueUnchanged)
{
  for (const double bad : {0.0, -1.0, std::nan(""), std::numeric_limits<double>::infinity()}) {
    const auto r = node_->set_parameter(rclcpp::Parameter("reverse_speed", bad));
    EXPECT_FALSE(r.successful) << "bad=" << bad;
    EXPECT_DOUBLE_EQ(node_->get_parameter("reverse_speed").as_double(), 0.5) << "bad=" << bad;
  }
}

TEST_F(CaSafetyNodeTest, IntegerCoercedToDouble)
{
  const auto r = node_->set_parameter(rclcpp::Parameter("stop_length", 8));
  EXPECT_TRUE(r.successful);
  EXPECT_EQ(node_->get_parameter("stop_length").as_int(), 8);
}

TEST_F(CaSafetyNodeTest, NonNumericRejected)
{
  const auto r =
    node_->set_parameter(rclcpp::Parameter("stop_length", std::string("close")));
  EXPECT_FALSE(r.successful);
}

TEST_F(CaSafetyNodeTest, BoolToggleApplied)
{
  const auto r = node_->set_parameter(rclcpp::Parameter("cancel_yaw_during_reverse", false));
  EXPECT_TRUE(r.successful);
  EXPECT_FALSE(node_->get_parameter("cancel_yaw_during_reverse").as_bool());
}

// With no obstacle source, the default passthrough policy must forward cmd_vel
// unchanged (the node never silently stops the boat for lack of data).
TEST_F(CaSafetyNodeTest, PassthroughWhenNoCloud)
{
  auto helper = std::make_shared<rclcpp::Node>("ca_safety_test_helper");
  auto pub = helper->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_smoothed", rclcpp::QoS(1));
  geometry_msgs::msg::TwistStamped received;
  bool got = false;
  auto sub = helper->create_subscription<geometry_msgs::msg::TwistStamped>(
    "piloting_mode/autonomous/cmd_vel", rclcpp::QoS(1),
    [&](const geometry_msgs::msg::TwistStamped & m) {received = m; got = true;});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(helper);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = 1.0;
  cmd.twist.angular.z = 0.5;

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!got && std::chrono::steady_clock::now() < deadline) {
    pub->publish(cmd);
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(got) << "no cmd_vel_out received";
  EXPECT_DOUBLE_EQ(received.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(received.twist.angular.z, 0.5);
}

// The viz timer must publish CollisionMonitorState (DO_NOTHING while clear) so
// CAMP's 2 s fill watchdog stays fed even when cmd_vel is idle.
TEST_F(CaSafetyNodeTest, PublishesStateOnTimer)
{
  auto helper = std::make_shared<rclcpp::Node>("ca_safety_test_helper");
  nav2_msgs::msg::CollisionMonitorState state;
  bool got = false;
  auto sub = helper->create_subscription<nav2_msgs::msg::CollisionMonitorState>(
    "collision_monitor_state", rclcpp::QoS(1),
    [&](const nav2_msgs::msg::CollisionMonitorState & m) {state = m; got = true;});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(helper);

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!got && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(got) << "no CollisionMonitorState received";
  EXPECT_EQ(state.action_type, nav2_msgs::msg::CollisionMonitorState::DO_NOTHING);
}

// Guard the safety-critical QoS choices against silent regression to a policy
// that would not match the reflex cloud (best-effort) — cf. #56.
TEST_F(CaSafetyNodeTest, PointcloudSubscriptionIsBestEffort)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::vector<rclcpp::TopicEndpointInfo> infos;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    infos = node_->get_subscriptions_info_by_topic("/collision_monitor/pointcloud");
    if (!infos.empty()) {break;}
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_FALSE(infos.empty()) << "no subscription on the pointcloud topic";
  bool best_effort = false;
  for (const auto & info : infos) {
    if (info.qos_profile().reliability() == rclcpp::ReliabilityPolicy::BestEffort) {
      best_effort = true;
    }
  }
  EXPECT_TRUE(best_effort) << "pointcloud sub must be best-effort to match the reflex cloud (#56)";
}
