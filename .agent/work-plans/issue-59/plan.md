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

## Approach

1. **Extract the corridor solver into `marine_nav_utilities`** — move
   `solveCorridorOffsets`, `makeLateralOffsets`, `resampleStations`, `Station`,
   `CorridorParams` out of `marine_nav_behavior_tree` into a pure
   `corridor_solver.{h,cpp}` (sibling to `costmap_window.cpp`). Move their unit
   tests. Point the existing BT node at the new header (no behavior change) so
   the extraction lands green on its own.
2. **New package `marine_nav_avoidance_controller`** — a decorator
   `nav2_core::Controller`:
   - `configure()`: nested `pluginlib` load of an **inner** controller (param:
     inner plugin id + its param namespace); cache `tf`, `costmap_ros`, node.
     Declare `survey_avoidance.*` tunables on the **controller_server lifecycle
     node** (externally param-serviceable — fixes the #57/15:03 wrong-node gap).
   - `setPlan(nominal)`: store the nominal line; forward to the inner controller.
   - `computeVelocityCommands(pose,…)`: read `costmap_ros_->getCostmap()`, run
     the solver **anchored at the boat's actual cross-track offset** (from
     `pose`), `setPlan(reshaped)` on the inner controller, delegate. Publish the
     operator overlay from this node.
   - `setSpeedLimit()` passthrough; realize `avoid_speed` by limiting the inner
     controller's speed during a deviation (drop the per-pose-stamp hack).
3. **Tests** — wrapper reshape-and-delegate against a fake inner controller +
   hand-built costmap; regression that the detour anchors at a non-zero actual
   offset (the bug #57 names); solver tests stay in utilities.
4. **BT wiring** — remove `AdjustPathForObstacles` from `SurveyLine` in
   `run_tasks.xml` (BT now passes the nominal line straight to FollowPath), and
   **delete** the BT node, its `bt_register_nodes` entry, `plugin_lib_names`
   entry, and its tests in this PR. Verify the new goal's `setPlan()` propagates
   through the wrapper to the inner follower (#35 mid-line re-send) — test it.
5. **Config rollout (cross-repo, before June 4)** — point the `FollowPath`
   controller plugin at the wrapper with crabbing as the inner, in
   `bizzyboat.yaml` (echoboats) and `nav2_params.base.yaml` (seafloor), with
   on-water validation before the survey.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_utilities/{include,src}/…/corridor_solver.{h,cpp}` | New: pure solver moved here |
| `marine_nav_utilities/test/test_corridor_solver.cpp` | Moved solver unit tests |
| `marine_nav_behavior_tree/.../adjust_path_for_obstacles.{h,cpp}` | Delete (node + registration + plugin_lib_names + tests) |
| `marine_nav_avoidance_controller/**` | New package: decorator controller + plugin.xml + tests |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Drop `AdjustPathForObstacles` from `SurveyLine` |
| `bizzyboat.yaml` / `nav2_params.base.yaml` *(echoboats / seafloor — separate PRs)* | FollowPath → wrapper(inner=crabbing) |

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
| Move solver to utilities | `marine_nav_behavior_tree` deps + includes; `package.xml` of both | Yes |
| Remove BT node from tree | `run_tasks.xml`, `bt_register_nodes`, `plugin_lib_names`, BT tests | Yes |
| New controller plugin | controller_server config in **echoboats + seafloor** repos | Yes — separate PRs, before June 4 |
| `avoid_speed` mechanism change | crabbing `setSpeedLimit` semantics | Yes (verify) |

## Decisions (resolved 2026-06-02)

- **Host shape**: decorator wrapper owning an inner `CrabbingPathFollower` (tracker stays pure).
- **Old BT node**: delete it in this PR (registration + `plugin_lib_names` + tests).
- **Package name**: `marine_nav_avoidance_controller`.
- **Timing**: land plugin **and** flip controller config before the **June 4 dev freeze**, with on-water validation before the June 15 survey. (Roland's call; accepted risk of a live control-path change close to class — mitigate with on-water validation, not just CI.)

## Open Questions

- Mid-line re-send (#35): confirm a new goal's `setPlan()` propagates through the wrapper to the inner — verify in code/test, don't assume.

## Estimated Scope

Core change is one PR in `unh_marine_navigation` (solver extraction + new plugin
package + tests + BT-node deletion); two dependent config PRs in echoboats and
seafloor, all before June 4. Solver extraction could be split as a first stacked
PR if review prefers.
