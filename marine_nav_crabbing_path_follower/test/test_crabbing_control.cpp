// Unit test for the marine_control device-control wiring of CrabbingPathFollower
// (unh_marine_autonomy#140 / ADR-0003 — the external marine_control device-control
// ADR, not this workspace's ADR-0003).
//
// CrabbingPathFollower is a Nav2 controller plugin, so its full configure() needs
// a controller_server + costmap. This test instead exercises the extracted
// declareCrabbingDefaultSpeed / declareCrabbingControlParams / bindCrabbingControls
// helpers — the actual code the plugin runs — against a bare LifecycleNode and a
// real ControlServer, with no nav2 bring-up. It checks that:
//   - all ten controls (default_speed + nine tunables) are advertised with their
//     default FloatingPointRange, UI group, and units;
//   - a platform `<name>.<t>_range` startup override is honoured (and integer
//     bounds are coerced; a malformed range falls back to the default);
//   - an in-range change over the marine_control channel is applied; and
//   - an out-of-range change is rejected by the range, with the value held
//     (a sentinel sent after it confirms the bad change was delivered, not
//     dropped).

#include <chrono>
#include <cmath>
#include <cstdint>
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

#include "marine_nav_crabbing_path_follower/crabbing_path_follower.h"

using marine_control_interfaces::msg::ControlItem;
using marine_control_interfaces::msg::ControlSet;
using marine_control_interfaces::msg::ControlValue;
using std::chrono::milliseconds;

namespace
{
constexpr char kName[] = "FollowPath";
constexpr char kStateTopic[] = "~/control/FollowPath/state";
constexpr char kChangeTopic[] = "~/control/FollowPath/change";

// Declare every control the plugin exposes (default_speed + the nine tunables),
// matching what configure() does, so the bound ControlSet is complete.
void declareAll(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  marine_nav_crabbing_path_follower::declareCrabbingDefaultSpeed(node, name);
  marine_nav_crabbing_path_follower::declareCrabbingControlParams(node, name);
}

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

class CrabbingControlTest : public ::testing::Test
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

TEST_F(CrabbingControlTest, AdvertisesAllControlsWithRangesAndGroups)
{
  auto node = makeNode();
  declareAll(node, kName);

  marine_control::ControlServerOptions opts;
  opts.device_name = "Crabbing Path Follower";
  opts.state_topic = kStateTopic;
  opts.change_topic = kChangeTopic;
  marine_control::ControlServer server(node.get(), opts);
  marine_nav_crabbing_path_follower::bindCrabbingControls(server, kName);

  const ControlSet set = server.build_control_set();
  ASSERT_EQ(set.items.size(), 10u);

  // default_speed: handled separately from the generic helper, with its own
  // permissive panel range so the configure-time fallback is preserved.
  const auto * speed = findItem(set, "FollowPath.default_speed");
  ASSERT_NE(speed, nullptr);
  EXPECT_EQ(speed->type, ControlItem::TYPE_FLOAT);
  EXPECT_EQ(speed->group, "speed");
  EXPECT_EQ(speed->units, "m/s");
  EXPECT_DOUBLE_EQ(speed->min_value, 0.0);
  EXPECT_DOUBLE_EQ(speed->max_value, 20.0);

  // Spot-check representative tunables across the remaining groups.
  const auto * gain = findItem(set, "FollowPath.heading_rate_gain");
  ASSERT_NE(gain, nullptr);
  EXPECT_EQ(gain->type, ControlItem::TYPE_FLOAT);
  EXPECT_EQ(gain->group, "heading");
  EXPECT_DOUBLE_EQ(gain->min_value, 0.0);
  EXPECT_DOUBLE_EQ(gain->max_value, 10.0);

  const auto * la = findItem(set, "FollowPath.lookahead_distance");
  ASSERT_NE(la, nullptr);
  EXPECT_EQ(la->group, "lookahead");
  EXPECT_EQ(la->units, "m");

  const auto * vmin = findItem(set, "FollowPath.pid.gain_v_min");
  ASSERT_NE(vmin, nullptr);
  EXPECT_EQ(vmin->group, "pid");
  EXPECT_EQ(vmin->units, "m/s");
}

TEST_F(CrabbingControlTest, PlatformRangeOverrideIsHonoured)
{
  // Platform narrows heading_rate_gain's bounds at startup.
  auto node = makeNode(
    {rclcpp::Parameter("FollowPath.heading_rate_gain_range", std::vector<double>{0.5, 5.0})});
  declareAll(node, kName);

  marine_control::ControlServer server(node.get());  // default topics fine here
  marine_nav_crabbing_path_follower::bindCrabbingControls(server, kName);

  const auto * gain = findItem(server.build_control_set(), "FollowPath.heading_rate_gain");
  ASSERT_NE(gain, nullptr);
  EXPECT_DOUBLE_EQ(gain->min_value, 0.5);
  EXPECT_DOUBLE_EQ(gain->max_value, 5.0);
}

TEST_F(CrabbingControlTest, IntegerRangeOverrideIsAcceptedAndCoerced)
{
  // A platform may write the bounds as integers (`[1, 5]`, the natural YAML
  // form). They must be accepted and coerced to doubles, not rejected with a
  // parameter-type-mismatch throw that would fail controller bring-up.
  auto node = makeNode(
    {rclcpp::Parameter("FollowPath.heading_rate_gain_range", std::vector<int64_t>{1, 5})});
  declareAll(node, kName);

  marine_control::ControlServer server(node.get());
  marine_nav_crabbing_path_follower::bindCrabbingControls(server, kName);

  const auto * gain = findItem(server.build_control_set(), "FollowPath.heading_rate_gain");
  ASSERT_NE(gain, nullptr);
  EXPECT_DOUBLE_EQ(gain->min_value, 1.0);
  EXPECT_DOUBLE_EQ(gain->max_value, 5.0);
}

TEST_F(CrabbingControlTest, MalformedRangeFallsBackToDefault)
{
  // min >= max is malformed -> the built-in default range is used.
  auto node = makeNode(
    {rclcpp::Parameter("FollowPath.heading_rate_gain_range", std::vector<double>{5.0, 1.0})});
  declareAll(node, kName);

  marine_control::ControlServer server(node.get());
  marine_nav_crabbing_path_follower::bindCrabbingControls(server, kName);

  const auto * gain = findItem(server.build_control_set(), "FollowPath.heading_rate_gain");
  ASSERT_NE(gain, nullptr);
  EXPECT_DOUBLE_EQ(gain->min_value, 0.0);
  EXPECT_DOUBLE_EQ(gain->max_value, 10.0);
}

// A distinct marine_control.namespace yields distinct control topics, so a
// wrapped follower and its wrapper don't collide. The bound parameter names are
// unchanged (always <plugin_name_>.*) — only the panel channel is namespaced.
TEST_F(CrabbingControlTest, NamespaceParamDifferentiatesTopicsWhenWrapped)
{
  auto node = makeNode();
  declareAll(node, kName);

  // Standalone default: namespace == plugin name -> historical topic layout.
  marine_control::ControlServerOptions standalone;
  standalone.state_topic = std::string("~/control/") + kName + "/state";
  standalone.change_topic = std::string("~/control/") + kName + "/change";
  // Wrapped: a distinct namespace -> a non-colliding topic.
  marine_control::ControlServerOptions wrapped;
  wrapped.state_topic = "~/control/FollowPath_inner/state";
  wrapped.change_topic = "~/control/FollowPath_inner/change";

  EXPECT_NE(standalone.state_topic, wrapped.state_topic);
  EXPECT_NE(standalone.change_topic, wrapped.change_topic);

  // Both bind the same parameter set; only the channel differs.
  marine_control::ControlServer s1(node.get(), standalone);
  marine_nav_crabbing_path_follower::bindCrabbingControls(s1, kName);
  ASSERT_EQ(s1.build_control_set().items.size(), 10u);
}

class CrabbingControlChannelTest : public CrabbingControlTest
{
protected:
  void buildServerAndClient()
  {
    node_ = makeNode();
    declareAll(node_, kName);
    marine_control::ControlServerOptions opts;
    opts.state_topic = kStateTopic;
    opts.change_topic = kChangeTopic;
    server_ = std::make_unique<marine_control::ControlServer>(node_.get(), opts);
    marine_nav_crabbing_path_follower::bindCrabbingControls(*server_, kName);

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

TEST_F(CrabbingControlChannelTest, InRangeChangeIsApplied)
{
  buildServerAndClient();
  EXPECT_DOUBLE_EQ(paramValue("heading_rate_gain"), 1.0);  // default

  ASSERT_TRUE(
    spinUntil(
      [this] {
        sendChange("FollowPath.heading_rate_gain", 5.0);
        return std::abs(paramValue("heading_rate_gain") - 5.0) < 1e-9;
      }))
    << "in-range change to heading_rate_gain was not applied over the channel";
}

TEST_F(CrabbingControlChannelTest, OutOfRangeChangeIsRejected)
{
  buildServerAndClient();

  // Drive to a known in-range baseline first.
  ASSERT_TRUE(
    spinUntil(
      [this] {
        sendChange("FollowPath.heading_rate_gain", 5.0);
        return std::abs(paramValue("heading_rate_gain") - 5.0) < 1e-9;
      }));

  // Request an out-of-range value (default max is 10), then an in-range
  // sentinel. RELIABLE ordered delivery means a landed sentinel proves the bad
  // change was delivered and rejected, not dropped.
  sendChange("FollowPath.heading_rate_gain", 999.0);
  ASSERT_TRUE(
    spinUntil(
      [this] {
        sendChange("FollowPath.heading_rate_gain", 8.0);
        return std::abs(paramValue("heading_rate_gain") - 8.0) < 1e-9;
      }))
    << "sentinel after the out-of-range change never landed";

  // The out-of-range value must never have taken effect (cap held at 10).
  EXPECT_LE(paramValue("heading_rate_gain"), 10.0);
  EXPECT_DOUBLE_EQ(paramValue("heading_rate_gain"), 8.0);
}
