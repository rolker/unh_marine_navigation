// Unit tests for SetControllerSpeed::resolveTargetNode — the pure helper
// that maps an XML-port target_node (absolute or relative) and a ROS
// namespace to the absolute node name the parameter client should target.

#include <gtest/gtest.h>

#include <string>

#include "marine_nav_behavior_tree/plugins/action/set_controller_speed.h"

using marine_nav_behavior_tree::SetControllerSpeed;

TEST(ResolveTargetNode, AbsoluteNamePassesThroughRegardlessOfNamespace)
{
  EXPECT_EQ(
    SetControllerSpeed::resolveTargetNode("/global/controller_server", "/bizzy"),
    "/global/controller_server");
  EXPECT_EQ(
    SetControllerSpeed::resolveTargetNode("/controller_server", "/"),
    "/controller_server");
  EXPECT_EQ(
    SetControllerSpeed::resolveTargetNode("/controller_server", ""),
    "/controller_server");
}

TEST(ResolveTargetNode, RelativeNameUnderSingleSegmentNamespace)
{
  // Bizzy-style: ns="/bizzy" + relative -> "/bizzy/controller_server"
  EXPECT_EQ(
    SetControllerSpeed::resolveTargetNode("controller_server", "/bizzy"),
    "/bizzy/controller_server");
}

TEST(ResolveTargetNode, RelativeNameUnderMultiSegmentNamespace)
{
  EXPECT_EQ(
    SetControllerSpeed::resolveTargetNode("controller_server", "/foo/bar"),
    "/foo/bar/controller_server");
}

TEST(ResolveTargetNode, RelativeNameUnderRootNamespaceDoesNotDoubleSlash)
{
  // rclcpp::Node::get_namespace() returns "/" for an unnamespaced node.
  // Naive concatenation would yield "//controller_server" — make sure we
  // collapse to a single leading slash.
  EXPECT_EQ(
    SetControllerSpeed::resolveTargetNode("controller_server", "/"),
    "/controller_server");
}

TEST(ResolveTargetNode, RelativeNameUnderEmptyNamespaceBehavesLikeRoot)
{
  // Defensive: some callers might pass "" instead of "/".
  EXPECT_EQ(
    SetControllerSpeed::resolveTargetNode("controller_server", ""),
    "/controller_server");
}
