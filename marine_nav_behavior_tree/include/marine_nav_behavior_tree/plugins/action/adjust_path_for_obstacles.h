#ifndef MARINE_NAV_BEHAVIOR_TREE_ACTIONS_ADJUST_PATH_FOR_OBSTACLES_H
#define MARINE_NAV_BEHAVIOR_TREE_ACTIONS_ADJUST_PATH_FOR_OBSTACLES_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>  // NOLINT(build/include_order) cpplint misclassifies third-party .h headers as "C system" by extension.
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "marine_nav_utilities/corridor_solver.h"

namespace marine_nav_behavior_tree
{

// The pure corridor solver (CorridorParams, Station, makeLateralOffsets,
// solveCorridorOffsets, resampleStations) lives in marine_nav_utilities so it
// can be shared with the controller-layer avoider (#59). Re-exported into this
// namespace so existing call sites in this node are unchanged.
using marine_nav_utilities::CorridorParams;
using marine_nav_utilities::makeLateralOffsets;
using marine_nav_utilities::resampleStations;
using marine_nav_utilities::solveCorridorOffsets;
using marine_nav_utilities::Station;

// Optionally slow the boat through the avoidance manoeuvre by writing per-pose
// timestamps on the contiguous deviating run of `path`. CrabbingPathFollower
// derives per-segment speed from segment_distance / Δstamp when both endpoint
// stamps are non-zero and increasing, so stamps spaced by distance/`avoid_speed`
// command `avoid_speed` (m/s) there. `offsets_d[i]` is the cross-track offset of
// `path.poses[i]`; |offset| >= `deviation_epsilon` marks a deviating pose. No-op
// when `avoid_speed` <= 0, sizes mismatch, or the deviating run is < 2 poses;
// poses outside the run keep their (zero) stamps so the controller uses its
// default speed. Timestamps are geometry-derived (fixed base, no wall clock) so a
// held detour does not re-trigger FollowPath preemption.
void applyAvoidanceSlowdown(
  nav_msgs::msg::Path & path, const std::vector<double> & offsets_d,
  double avoid_speed, double deviation_epsilon);

// Shared costmap cache. Held by both the node and the subscription callback so
// that an in-flight callback running on the executor thread keeps the cache
// alive even if the BT node is destroyed mid-callback (tree reload / shutdown).
// The callback captures a shared_ptr to this — never the node's `this`.
struct CostmapCache
{
  std::mutex mutex;
  std::shared_ptr<nav2_msgs::msg::Costmap> costmap;
};

// Reactive corridor path-adjuster. Sits between SetPathFromTask and FollowPath
// in the SurveyLine inner ReactiveSequence: reads the nominal trackline, samples
// the existing local costmap, and reshapes the followed path around obstacles
// while staying anchored to the line. Pure tracker downstream (CrabbingPathFollower)
// is untouched. When clear it passes the nominal path through unchanged; when the
// corridor is fully blocked it also passes nominal through, degrading to the
// existing reflex-stop / RecoveryNode path. Always returns SUCCESS — it is a path
// transformer, not a gate. See .agent/work-plans/issue-30/plan.md.
class AdjustPathForObstacles : public BT::SyncActionNode
{
public:
  AdjustPathForObstacles(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // Latest costmap, swapped in by the subscription callback (executor thread)
  // and read by tick() (BT thread). Lives in a shared cache the callback owns a
  // reference to (see CostmapCache) so callback and node lifetimes are decoupled.
  // tick() takes a shared_ptr snapshot under the lock and releases it before the DP.
  std::shared_ptr<CostmapCache> costmap_cache_ = std::make_shared<CostmapCache>();
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  std::string costmap_topic_;

  // Operator-feedback markers (auto-discovered by CAMP): nominal line, adjusted
  // path, the deviating "avoiding" band, and a text flag. Published only while
  // deviating; cleared with a DELETEALL on the avoiding->clear transition, which
  // was_avoiding_ tracks so an idle tree doesn't republish every tick.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  bool was_avoiding_ = false;

  // Previous tick's per-station offsets, for the temporal (chatter) term.
  std::vector<double> prev_offsets_;
};

}  // namespace marine_nav_behavior_tree

#endif  // MARINE_NAV_BEHAVIOR_TREE_ACTIONS_ADJUST_PATH_FOR_OBSTACLES_H
