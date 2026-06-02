# Plan: Move survey-line obstacle avoidance into a controller decorator (off the BT node)

## Issue

https://github.com/rolker/unh_marine_navigation/issues/59

## Context

Survey-line avoidance currently lives in the BT node `AdjustPathForObstacles`
(#30), which owns its own `costmap_raw` subscription on the bt_navigator's
internal rclcpp node and rewrites `{survey_path}` every tick. Field RCA (#57,
deployment echoboats#201) traced the "doesn't avoid / only updates behind the
boat" symptom to three *hosting* artifacts — costmap staleness (separate sub +
weak QoS + shared executor), FollowPath preemption storm (path churn at ~7 Hz),
and a near-anchor pinned to `d=0` at the boat. The corridor solver itself is
sound. `CrabbingPathFollower` is already a `nav2_core::Controller` handed a live
`Costmap2DROS` (`crabbing_path_follower.h:28,49`), so the controller layer is the
correct host: the framework delivers a fresh, in-process costmap there.

**Scope note (what moves vs. what is new):** only the *pure* solver
(`solveCorridorOffsets`, `makeLateralOffsets`, `resampleStations`, `Station`,
`CorridorParams`) is a move — it's already ROS-free and unit-tested. The
costmap **sampling**, TF resolution, active-range computation, and the
near-anchor fix are **net-new controller code**: the BT node sampled a
`nav2_msgs/Costmap` *message*; the controller samples `costmap_2d::Costmap2D`
via `getCost()/worldToMap()`. So this is "move the math, re-implement the
plumbing," not a straight re-host.

**Single combined PR #60** (Roland's call — fewer PR mechanics against the June 4
clock). Built in two logical phases / atomic commits below; both land on
`feature/issue-59`, and the PR `Closes #59`. The echoboats config flip is the one
separate PR (different repo).

### Phase 1 — extract the solver (atomic commit, green on its own)
1. Move the pure solver + its unit tests from `marine_nav_behavior_tree` into
   `marine_nav_utilities` as `corridor_solver.{h,cpp}` (sibling to
   `costmap_window.cpp`). Point the existing BT node at the new header — no
   behavior change. Update both packages' `CMakeLists.txt` + `package.xml`.

### Phase 2 — decorator controller + delete the BT node
2. **New package `marine_nav_avoidance_controller`** — a decorator
   `nav2_core::Controller`:
   - `configure()`: nested `pluginlib` load of an **inner** controller (param:
     inner plugin id + its param namespace); cache `tf`, `costmap_ros`, node.
     **The `pluginlib::ClassLoader` for the inner controller must be a member
     that outlives the loaded instance** — a scope-local loader unloads the
     symbol and crashes on use. Declare `survey_avoidance.*` tunables on the
     **controller_server lifecycle node** (externally param-serviceable — fixes
     the #57/15:03 wrong-node gap). Drive the inner controller's full lifecycle
     (configure/activate/deactivate/cleanup) from the wrapper's.
   - `setPlan(nominal)`: store the nominal line; forward to the inner controller.
   - `computeVelocityCommands(pose,…)`: read `costmap_ros_->getCostmap()`, run
     the solver **anchored at the boat's actual cross-track offset** (from
     `pose`), `setPlan(reshaped)` on the inner controller, delegate. Publish the
     operator overlay from this node.
   - `setSpeedLimit()` passthrough. **`avoid_speed` decision (resolve at impl):**
     the BT design slowed *only the deviating segment* (per-pose stamps);
     routing it through the inner `setSpeedLimit` slows the *whole controller*
     while any deviation is active. Default to whole-controller toggle for v1
     (simpler, restore on clear) unless per-segment proves necessary — flag to
     Roland when reached.
3. **Tests** — wrapper reshape-and-delegate against a fake inner controller +
   hand-built `Costmap2D`; regression that the detour anchors at a non-zero
   actual offset (the bug #57 names); `setPlan()` re-send propagates to the inner
   (#35); corridor-blocked → delegate-to-inner-on-nominal. Solver tests stay in
   utilities (PR1).
4. **BT wiring + deletion** — remove `AdjustPathForObstacles` from `SurveyLine`
   in `run_tasks.xml` (BT now passes the nominal line straight to FollowPath),
   and **delete** the node (`adjust_path_for_obstacles.{h,cpp}`), its
   `bt_register_nodes` entry, its `CMakeLists.txt` source (:74) + test target
   (:306-316), and `test/test_adjust_path_for_obstacles.cpp`. (No
   `plugin_lib_names` change — the node compiles into the
   `marine_nav_behavior_tree` BT-plugins lib, which stays listed; only one node
   is removed from it.)
5. **Config flip (echoboats overlay only, before June 4)** — override the
   `FollowPath` controller `plugin:` to the wrapper (inner = crabbing) in
   **`bizzyboat_project11/config/nav2_overlay.yaml`** (the per-instance overlay),
   *not* the shared `nav2_params.base.yaml`. This flips BizzyBoat alone and
   leaves seafloor/ben on crabbing-direct. On-water validation before the survey.

## Files to Change

| File | Phase | Change |
|------|-------|--------|
| `marine_nav_utilities/{include,src}/…/corridor_solver.{h,cpp}` | 1 ✅ | New: pure solver moved here |
| `marine_nav_utilities/test/test_corridor_solver.cpp` + `CMakeLists.txt`/`package.xml` | 1 ✅ | Moved solver unit tests + build wiring |
| `marine_nav_behavior_tree/.../adjust_path_for_obstacles.{h,cpp}` | 1→2 | Ph1 ✅: include solver from utilities (no behavior change). Ph2: delete |
| `marine_nav_behavior_tree/CMakeLists.txt` (:74 src, test block) + `bt_register_nodes.cpp` + `test/test_adjust_path_for_obstacles.cpp` | 2 | Remove node source, test target, registration |
| `marine_nav_avoidance_controller/**` (`CMakeLists.txt`, `package.xml`, `plugin.xml`, src, tests) | 2 | New decorator-controller package |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | 2 | Drop `AdjustPathForObstacles` from `SurveyLine` |
| `bizzyboat_project11/config/nav2_overlay.yaml` *(echoboats — separate PR)* | cfg | Override `FollowPath` plugin → wrapper(inner=crabbing), BizzyBoat-only |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Robustness over open water | Fresh in-process costmap by construction; removes the staleness + preemption-churn failure modes rather than instrumenting around them |
| Fix completely (test + edge + lifecycle) | Anchor-at-actual-offset regression; wrapper lifecycle (configure/activate/cleanup) exercised; corridor-blocked → delegate-to-inner-on-nominal preserved |
| Don't circumvent tests | Reuse the existing pure-solver tests via the extraction; no test config changes |
| Workspace grep before API changes | Solver symbols are public in `marine_nav_behavior_tree`; grep workspace for external users before moving (incl. `bt_register_nodes`, any tests) |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (ROS 2 conventions) | Yes | A decorator `nav2_core::Controller` + nested pluginlib is the idiomatic Nav2 pattern; params on the lifecycle node are externally serviceable |
| ADR-0002 (worktree isolation) | Yes | Work in `feature/issue-59` worktree |

## Consequences

| If we change… | Also update… | Included? |
|---|---|---|
| Move solver to utilities | `marine_nav_behavior_tree` + `marine_nav_utilities` `CMakeLists.txt`/`package.xml`; BT-node include | Yes (PR1) |
| Remove BT node from tree | `run_tasks.xml`, `bt_register_nodes.cpp`, `CMakeLists.txt` (src + test), BT test file | Yes (PR2) — no `plugin_lib_names` change (lib stays) |
| Flip BizzyBoat controller | `nav2_overlay.yaml` override only (NOT shared `nav2_params.base.yaml`) — leaves seafloor/ben on crabbing-direct | Yes (echoboats cfg PR) |
| `avoid_speed` mechanism change | inner `setSpeedLimit` is whole-controller, not per-segment — behavior shift | Decide at impl (default whole-controller v1) |

## Decisions (resolved 2026-06-02)

- **Host shape**: decorator wrapper owning an inner `CrabbingPathFollower` (tracker stays pure).
- **Old BT node**: delete it (PR2) — node source, `bt_register_nodes` entry, `CMakeLists.txt` src+test, BT test file. (No `plugin_lib_names` change — the BT-plugins lib stays listed.)
- **Package name**: `marine_nav_avoidance_controller`.
- **Timing**: land plugin **and** flip controller config before the **June 4 dev freeze**, with on-water validation before the June 15 survey. (Roland's call; accepted risk of a live control-path change close to class — mitigate with on-water validation, not just CI.)

## Acceptance Criteria

- **Sim**: with a live local costmap, the followed path's *ahead* portion
  re-plans as obstacles appear/clear ahead of the boat (#57's desired outcome),
  and a deviation relaxes to nominal once the obstacle leaves the costmap.
- **Sim**: the boat actually tracks the detour around a stationary obstacle (the
  near-anchor no longer pins it to the line) — the #59 "never went around" fix.
- **On-water** (before June 15): BizzyBoat bends around a real obstacle and the
  CAMP overlay tracks the live costmap, not a line-start snapshot.
- All existing crabbing-follower behavior unchanged when no obstacle is present
  (wrapper passes through to the inner controller).

## Open Questions

- Mid-line re-send (#35): confirm a new goal's `setPlan()` propagates through the wrapper to the inner — verify in code/test, don't assume.
- `avoid_speed`: whole-controller slowdown (v1 default) vs. per-segment — confirm with Roland at PR2 impl.

## Estimated Scope

**One PR** (#60, `unh_marine_navigation`, `Closes #59`): solver extraction +
new `marine_nav_avoidance_controller` package + tests + BT-node deletion +
`run_tasks.xml`, built in two atomic-commit phases.
**Config PR** (`unh_echoboats_project11`): `nav2_overlay.yaml` controller-plugin
override (BizzyBoat-only). All before June 4; seafloor/ben need no change.
