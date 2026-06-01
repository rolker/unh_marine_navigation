// Regression test for #48: the hover visualization publisher must be created at
// lifecycle onConfigure (so the DDS endpoint exists before operator clients
// subscribe), not lazily at the first hover, and torn down on onCleanup.
//
// This exercises the lifecycle hooks directly (no costmap/tf fixture needed):
// a test subclass binds a LifecycleNode + behavior name, calls onConfigure /
// onCleanup, and asserts on the protected publisher pointer — deterministic, no
// dependence on DDS discovery timing.

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "marine_nav_behaviors/hover.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

// Exposes the protected publisher pointer and lets the test bind a node +
// behavior name without running the full TimedBehavior::configure() (which
// needs tf + costmap collision checkers we don't exercise here).
class HoverProbe : public marine_nav_behaviors::Hover
{
public:
  void bind(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
  {
    node_ = node;
    behavior_name_ = "hover";
    logger_ = node->get_logger();
  }

  bool hasVizPublisher() const { return static_cast<bool>(visualization_publisher_); }
};

rclcpp_lifecycle::LifecycleNode::SharedPtr makeNode(const std::string & name, bool generate_viz)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name);
  // Pre-declare so onConfigure's declare_parameter_if_not_declared is a no-op
  // and its get_parameter reads this value.
  node->declare_parameter("hover.generate_visualization", generate_viz);
  return node;
}

}  // namespace

// generate_visualization=true: publisher exists immediately after onConfigure
// (before any hover), and is gone after onCleanup.
TEST(HoverVisualization, PublisherCreatedAtConfigureAndTornDownAtCleanup)
{
  auto node = makeNode("hover_viz_enabled", true);
  HoverProbe hover;
  hover.bind(node);

  EXPECT_FALSE(hover.hasVizPublisher()) << "publisher should not exist before configure";
  hover.onConfigure();
  EXPECT_TRUE(hover.hasVizPublisher()) << "publisher must exist right after onConfigure (#48)";
  hover.onCleanup();
  EXPECT_FALSE(hover.hasVizPublisher()) << "publisher must be reset after onCleanup";
}

// generate_visualization=false (the default): no publisher is created.
TEST(HoverVisualization, NoPublisherWhenDisabled)
{
  auto node = makeNode("hover_viz_disabled", false);
  HoverProbe hover;
  hover.bind(node);

  hover.onConfigure();
  EXPECT_FALSE(hover.hasVizPublisher()) << "no publisher should be created when disabled";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
