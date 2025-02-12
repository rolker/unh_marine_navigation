#include "project11_navigation/plugins/action/debug_blackboard.h"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::DebugBlackboardString>("DebugBlackboardString");
  factory.registerNodeType<project11_navigation::DebugBlackboardDouble>("DebugBlackboardDouble");
}
