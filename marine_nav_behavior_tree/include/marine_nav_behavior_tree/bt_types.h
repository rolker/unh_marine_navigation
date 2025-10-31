#ifndef MARINE_NAV_BEHAVIOR_TREE_BT_TYPES_H
#define MARINE_NAV_BEHAVIOR_TREE_BT_TYPES_H

#include "std_msgs/msg/color_rgba.hpp"
#include "behaviortree_cpp/basic_types.h"

namespace BT
{
  template <> inline std_msgs::msg::ColorRGBA convertFromString(StringView str)
  {
    auto parts = splitString(str, ',');
    if(parts.size() != 4)
    {
      throw RuntimeError("Invalid input, expecting 4 color values (r, g, b, a)");
    }
    std_msgs::msg::ColorRGBA color;
    color.r = convertFromString<double>(parts[0]);
    color.g = convertFromString<double>(parts[1]);
    color.b = convertFromString<double>(parts[2]);
    color.a = convertFromString<double>(parts[3]);

    return color;
  }
} // end namespace BT


namespace marine_nav_behavior_tree
{

void registerJsonDefinitions();


} // namespace marine_nav_behavior_tree

#endif

