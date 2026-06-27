// Unit test for the marine_control device-control wiring of AvoidanceController
// (unh_marine_autonomy#140 / ADR-0003).
//
// AvoidanceController is a Nav2 controller plugin, so its full configure() needs
// a controller_server + costmap + inner controller. This test instead exercises
// the extracted declareAvoidanceControlParams / bindAvoidanceControls helpers —
// the actual code the plugin runs — against a bare LifecycleNode and a real
// ControlServer, with no nav2 bring-up. It checks that:
//   - all 10 tunables are advertised with their default FloatingPointRange,
//     UI group, and units;
//   - a platform `<name>.<t>_range` startup override is honoured;
//   - an in-range change over the marine_control channel is applied; and
//   - an out-of-range change is rejected by the range, with the value held
//     (a sentinel sent after it confirms the bad change was delivered, not
//     dropped).

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "marine_control/control_server.hpp"
#include "marine_control_interfaces/msg/control_item.hpp"
#include "marine_control_interfaces/msg/control_set.hpp"
#include "marine_control_interfaces/msg/control_value.hpp"

#include "marine_nav_avoidance_controller/avoidance_controller.h"

using marine_control_interfaces::msg::ControlItem;
using marine_control_interfaces::msg::ControlSet;
using marine_control_interfaces::msg::ControlValue;
using std::chrono::milliseconds;

namespace
{
constexpr char kName[] = "FollowPath";
constexpr char kStateTopic[] = "~/control/FollowPath/state";
constexpr char kChangeTopic[] = "~/control/FollowPath/change";

const ControlItem * findItem(const ControlSet & set, const std::string & name)
{
  for (const auto & item : set.items) {
    if (item.name == name) {
      return &item;
    }
  }
  return nullptr;
}

rclcpp::QoS controlQos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

// Initialise rclcpp once for the whole program, before any fixture is
// constructed — a fixture member executor would otherwise build its guard
// condition against an invalid context if init ran per-test in SetUp().
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
[[maybe_unused]] ::testing::Environment * const kRclcppEnv =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);
}  // namespace

class AvoidanceControlTest : public ::testing::Test
{
protected:
  // Make a controller_server stand-in node, optionally with parameter overrides
  // (used to simulate a platform setting a custom <name>.<t>_range at startup).
  rclcpp_lifecycle::LifecycleNode::SharedPtr makeNode(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    rclcpp::NodeOptions options;
    if (!overrides.empty()) {
      options.parameter_overrides(overrides);
    }
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>("ctrl_srv", options);
  }
};

TEST_F(AvoidanceControlTest, AdvertisesAllTunablesWithRangesAndGroups)
{
  auto node = makeNode();
  marine_nav_avoidance_controller::declareAvoidanceControlParams(node, kName);

  marine_control::ControlServerOptions opts;
  opts.device_name = "Survey Obstacle Avoidance";
  opts.state_topic = kStateTopic;
  opts.change_topic = kChangeTopic;
  marine_control::ControlServer server(node.get(), opts);
  marine_nav_avoidance_controller::bindAvoidanceControls(server, kName);

  const ControlSet set = server.build_control_set();
  ASSERT_EQ(set.items.size(), 10u);

  // Spot-check representative controls across all three groups.
  const auto * max_dev = findItem(set, "FollowPath.max_deviation");
  ASSERT_NE(max_dev, nullptr);
  EXPECT_EQ(max_dev->type, ControlItem::TYPE_FLOAT);
  EXPECT_EQ(max_dev->group, "geometry");
  EXPECT_EQ(max_dev->units, "m");
  EXPECT_DOUBLE_EQ(max_dev->min_value, 0.1);
  EXPECT_DOUBLE_EQ(max_dev->max_value, 100.0);

  const auto * weight = findItem(set, "FollowPath.obstacle_avoidance_weight");
  ASSERT_NE(weight, nullptr);
  EXPECT_EQ(weight->group, "weights");
  EXPECT_DOUBLE_EQ(weight->min_value, 0.0);
  EXPECT_DOUBLE_EQ(weight->max_value, 1000.0);

  const auto * speed = findItem(set, "FollowPath.avoid_speed");
  ASSERT_NE(speed, nullptr);
  EXPECT_EQ(speed->group, "speed");
  EXPECT_EQ(speed->units, "m/s");
}

TEST_F(AvoidanceControlTest, PlatformRangeOverrideIsHonoured)
{
  // Platform narrows max_deviation's bounds at startup.
  auto node = makeNode(
    {rclcpp::Parameter("FollowPath.max_deviation_range", std::vector<double>{1.0, 10.0})});
  marine_nav_avoidance_controller::declareAvoidanceControlParams(node, kName);

  marine_control::ControlServer server(node.get());  // default topics fine here
  marine_nav_avoidance_controller::bindAvoidanceControls(server, kName);

  const auto * max_dev = findItem(server.build_control_set(), "FollowPath.max_deviation");
  ASSERT_NE(max_dev, nullptr);
  EXPECT_DOUBLE_EQ(max_dev->min_value, 1.0);
  EXPECT_DOUBLE_EQ(max_dev->max_value, 10.0);
}

TEST_F(AvoidanceControlTest, MalformedRangeFallsBackToDefault)
{
  // min >= max is malformed -> the built-in default range is used.
  auto node = makeNode(
    {rclcpp::Parameter("FollowPath.max_deviation_range", std::vector<double>{50.0, 5.0})});
  marine_nav_avoidance_controller::declareAvoidanceControlParams(node, kName);

  marine_control::ControlServer server(node.get());
  marine_nav_avoidance_controller::bindAvoidanceControls(server, kName);

  const auto * max_dev = findItem(server.build_control_set(), "FollowPath.max_deviation");
  ASSERT_NE(max_dev, nullptr);
  EXPECT_DOUBLE_EQ(max_dev->min_value, 0.1);
  EXPECT_DOUBLE_EQ(max_dev->max_value, 100.0);
}

class AvoidanceControlChannelTest : public AvoidanceControlTest
{
protected:
  void buildServerAndClient()
  {
    node_ = makeNode();
    marine_nav_avoidance_controller::declareAvoidanceControlParams(node_, kName);
    marine_control::ControlServerOptions opts;
    opts.state_topic = kStateTopic;
    opts.change_topic = kChangeTopic;
    server_ = std::make_unique<marine_control::ControlServer>(node_.get(), opts);
    marine_nav_avoidance_controller::bindAvoidanceControls(*server_, kName);

    client_ = std::make_shared<rclcpp::Node>("test_client");
    change_pub_ = client_->create_publisher<ControlValue>(
      "/ctrl_srv/control/FollowPath/change", controlQos());

    exec_.add_node(node_->get_node_base_interface());
    exec_.add_node(client_);
  }

  void sendChange(const std::string & name, double value)
  {
    ControlValue msg;
    msg.name = name;
    msg.value = std::to_string(value);
    change_pub_->publish(msg);
  }

  double paramValue(const std::string & suffix)
  {
    return node_->get_parameter(std::string(kName) + "." + suffix).as_double();
  }

  // Spin both nodes until `predicate()` holds or the timeout elapses.
  bool spinUntil(std::function<bool()> predicate, milliseconds timeout = milliseconds(2000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_some();
      if (predicate()) {
        return true;
      }
      rclcpp::sleep_for(milliseconds(10));
    }
    return predicate();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<marine_control::ControlServer> server_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Publisher<ControlValue>::SharedPtr change_pub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

TEST_F(AvoidanceControlChannelTest, InRangeChangeIsApplied)
{
  buildServerAndClient();
  EXPECT_DOUBLE_EQ(paramValue("max_deviation"), 6.0);  // default

  ASSERT_TRUE(
    spinUntil(
      [this] {
        sendChange("FollowPath.max_deviation", 8.0);
        return std::abs(paramValue("max_deviation") - 8.0) < 1e-9;
      }))
    << "in-range change to max_deviation was not applied over the channel";
}

TEST_F(AvoidanceControlChannelTest, OutOfRangeChangeIsRejected)
{
  buildServerAndClient();

  // Drive to a known in-range baseline first.
  ASSERT_TRUE(
    spinUntil(
      [this] {
        sendChange("FollowPath.max_deviation", 8.0);
        return std::abs(paramValue("max_deviation") - 8.0) < 1e-9;
      }));

  // Request an out-of-range value (default max is 100), then an in-range
  // sentinel. RELIABLE ordered delivery means a landed sentinel proves the bad
  // change was delivered and rejected, not dropped.
  sendChange("FollowPath.max_deviation", 999.0);
  ASSERT_TRUE(
    spinUntil(
      [this] {
        sendChange("FollowPath.max_deviation", 50.0);
        return std::abs(paramValue("max_deviation") - 50.0) < 1e-9;
      }))
    << "sentinel after the out-of-range change never landed";

  // The out-of-range value must never have taken effect (cap held at 100).
  EXPECT_LE(paramValue("max_deviation"), 100.0);
  EXPECT_DOUBLE_EQ(paramValue("max_deviation"), 50.0);
}
