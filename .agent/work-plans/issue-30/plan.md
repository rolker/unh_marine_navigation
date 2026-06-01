# Plan: Plan around obstacles while running survey lines

## Issue

https://github.com/rolker/unh_marine_navigation/issues/30

> **Reframing — supersedes the issue body.** Issue #30 proposes pre-decomposing
> tracklines into short, individually skippable segments plus a deferred-segment
> set, a return-pass scheduler, and coverage bookkeeping. Design discussion
> converged on a simpler approach with the same goal — keep covering a survey
> line around an obstacle instead of losing/abandoning it — and far less
> machinery. This plan implements that approach. **Action for a human:** the #30
> body still describes the old segment-decomposition design; reconcile the issue
> text (or close-as-superseded and re-title) so the tracker matches.

## Context

Verified against the deployed stack:

- **Survey lines bypass the planner.** The `SurveyLine` subtree in
  `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` runs
  `SetPathFromTask` → `FollowPath` directly. `SmacPlannerHybrid` is only used by
  `GotoPose`/`NavigateThroughWaypoints`. So obstacle-avoidance cannot be added by
  tuning the planner — the trackline goes straight to the controller as a fixed path.
- **Controller is a pure tracker** — `marine_nav_crabbing_path_follower::CrabbingPathFollower`
  (minimizes XTE via crab angle). Keep it a pure tracker; do not load avoidance
  into it (consistent with the #29 split).
- **The obstacle field already exists** as `local_costmap/costmap_raw`
  (`nav2_msgs/Costmap`, 0–254): rolling 400×400 @ 0.25 m = 100 m robot-centred
  window, frame `<tf_prefix>/map_tide`, layers `chart`(S57) + sea_surface +
  obstacle + inflation (`inflation_radius: 150.0`).
  Config: `seafloor_echoboat_project11/.../config/nav2_params.base.yaml`.
- BT nodes obtain an `rclcpp::Node` from blackboard key `"node"` (see
  `set_controller_speed.cpp`, `set_task_done.cpp`).

## Approach

Add one reactive BT node that reshapes the survey path around obstacles **before**
it reaches `FollowPath`, as a Frenet-frame corridor optimizer where the cross-track
offset `d` **is** the cross-track error — so the XTE term is analytic (`w_xte·d²`),
never rasterized, and never competes with the costmap's `uint8` budget. Obstacle
cost is *sampled* from the existing costmap; no new grid is created anywhere.

1. **`AdjustPathForObstacles` (BT `SyncActionNode`)** in `marine_nav_behavior_tree`.
   Ports: `nominal_path`(in `Path`), `path`(out `Path`), `costmap_topic`
   (default `local_costmap/costmap_raw`), `max_xte`, `lateral_step`(~0.5 m),
   `station_step`(~2 m), `w_xte`, `w_obs`, `w_smooth`, `w_temporal`,
   `max_lateral_rate`. Constructor grabs `node` from the blackboard, opens a
   costmap subscription + a TF buffer. **Subscription is sound (resolved):** the
   blackboard `node` is nav2's `BtActionServer` client node, externally spun on a
   dedicated executor thread (`set_controller_speed.cpp` relies on exactly this),
   so the cached-subscription callback fires. It is, however, the first
   subscription-bearing BT node in this repo. The callback runs on the
   bt_navigator executor thread — **distinct** from the BT tick thread — so the
   cache mutex is load-bearing: store the latest costmap as a `shared_ptr` and do a
   pointer-swap under the lock; the DP reads its own snapshot lock-free (don't hold
   the lock across the O(stations·offsets²) loop).
   **`max_xte` (resolved):** node param is the baseline; an optional per-line
   override is read from the task when present. Wire only the node side now
   (param + read-override-if-present) — no task-message / CAMP plumbing in this PR.
2. **tick()** — pass-through fast path when `nominal_path` is empty or no costmap
   yet. Otherwise: resample nominal into stations that are **ahead of the boat and
   inside the costmap window**; clamp first & last active station to `d=0` (anchor
   ends → a true detour that returns to the line). For each station, offsets
   `d ∈ [−max_xte, +max_xte]` at `lateral_step`; world point `on_line(s) + d·left_normal(s)`
   built in the nominal path's frame, then **transformed into the costmap frame
   before sampling** (default to transforming; short-circuit only when the path's
   `frame_id` already equals the costmap frame — `TaskInformation.poses` carry the
   publisher's frame, not a forced `map_tide`); `node_cost = INF` if at/above inscribed/lethal else
   `w_xte·d² + w_obs·g(cost)`, with `g(cost)` the **raw graded cost — no baseline
   subtraction, no threshold** (resolved). The `w_xte·d²` term is a restoring
   spring on the line, and a per-station *uniform* obstacle-cost offset does not
   move the DP's argmin — only the obstacle-cost **gradient** across the corridor
   does. So the 150 m inflation tails can't bow the line; it deviates only where
   cost steepens near a real hazard enough to overcome the spring. Tune the single
   `w_xte : w_obs` ratio (against the inflation `cost_scaling_factor`); the `INF`
   floor guarantees it never plans into the obstacle body regardless of the spring.
3. **DP over stations** (this is the whole "planner"): transition cost
   `w_smooth·(dᵢ−dₖ)² + w_temporal·(dᵢ−d_prev_tick(sᵢ))²`, pruning transitions
   that exceed `max_lateral_rate`. O(stations·offsets²), microseconds. Backtrack
   `d(s)`, weld reshaped stations onto the untouched tail, set headings along the
   path tangent, mirror `buildPath`'s zero-outer-stamp idiom (#23). Per-pose stamps
   are **not** load-bearing for `CrabbingPathFollower` (it times off the robot pose
   stamp and selects segments geometrically), so resampled stations — which have no
   meaningful original stamp — need not carry one; pass through the tail's stamps as
   hygiene only. Always returns `SUCCESS` — it's a transformer, not a gate.
4. **Blocked corridor → output nominal unchanged.** If min DP cost is `INF` (or no
   improvement beyond a republish deadband), emit the nominal path. This degrades
   to today's behaviour: Collision-Monitor reflex stop → `FollowPath` ABORT →
   existing `RecoveryNode` → `SetTaskFailed` → pilot. No new blockage logic.
5. **Wire into the BT — the *right* tree.** `run_tasks.xml` has two survey trees;
   target the executed one: `BehaviorTree ID="SurveyLine"` (≈:389), whose **inner
   `<ReactiveSequence>`** (≈:397) holds the `SetPathFromTask` that writes
   `{survey_path}` (≈:410) and the `RecoveryNode`-wrapped `FollowPath` reading
   `{survey_path}` (≈:423). Rename **that** `SetPathFromTask`'s output to
   `{nominal_survey_path}`; insert `AdjustPathForObstacles` reading
   `{nominal_survey_path}` → writing `{survey_path}`; leave `FollowPath` untouched.
   Do **not** touch the separate `BehaviorTree ID="SurveyLineTask"` (≈:487) or its
   `SetPathFromTask` (≈:491) — confirm during impl it isn't a second
   survey-following site before deciding it needs no change. The inner
   `ReactiveSequence` re-tick IS the monitor — a new obstacle reshapes the path
   next loop and `FollowPath` preempts on the changed `{survey_path}` (the #35
   `on_wait_for_result` mechanism). A `SyncActionNode` returning `SUCCESS` each
   tick preserves the reactive re-read semantics. No separate watcher node.
6. **Register + Groot palette** — register the node in `bt_register_nodes.cpp`
   (this regenerates the build-time `marine_nav_behavior_tree_nodes.xml` manifest
   — do **not** hand-edit that generated file). Hand-add the node to the two
   maintained-by-hand model lists: the `nav2.btproj` `<TreeNodesModel>` (Groot
   palette) and the inline `<TreeNodesModel>` inside `run_tasks.xml` (≈:544-822),
   with its `<Action>` ports.
7. **Unit-test the DP as a free function** (no ROS), `test_set_path_from_task.cpp`
   style: clear costmap → output identical to nominal; single blob → bounded
   detour that re-anchors to the line at both ends; full wall → pass-through.

### Scope for v1 (Roland's call)

No gap tracking, no coverage-state node, no deferred list, no return scheduler.
The pilot handles any holes — which makes **operator visibility** the substitute:
publish the nominal and deviated paths so the bulge is eyeballable. The display
work lives in `rolker/unh_echoboats_project11#183` (CA-awareness CAMP
overlay/annunciator), not here; this plan only ensures both paths are observable.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_behavior_tree/src/plugins/action/adjust_path_for_obstacles.cpp` | New node: subscription, Frenet lattice + DP, tick() |
| `marine_nav_behavior_tree/include/.../adjust_path_for_obstacles.h` | New header; DP exposed as a free function for testing |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register `AdjustPathForObstacles` |
| `marine_nav_behavior_tree/CMakeLists.txt` | Add `.cpp` to the `bt_plugins` lib + a new `ament_add_gtest` block (nodes manifest regenerates — not hand-edited) |
| `marine_nav_bt_task_navigator/behavior_trees/nav2.btproj` | Add node to Groot `<TreeNodesModel>` palette |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Insert node + rename one port in inner `SurveyLine` ReactiveSequence (≈:397/410); add `<Action>` to inline `<TreeNodesModel>` (≈:544-822) |
| `marine_nav_behavior_tree/test/test_adjust_path_for_obstacles.cpp` | New: clear / blob / wall cases |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | v1 drops the coverage node / deferred list / scheduler from #30; one BT node + one XML edit. Minimal corridor DP, not Hybrid-A*. |
| Test what breaks | DP is a pure free function with clear/blob/wall regression tests; covers the failure mode (obstacle on the line) directly. |
| A change includes its consequences | Node registration, manifest, CMake test target, BT wiring, and the #183 visibility dependency are all in-plan. |
| Capture decisions, not just implementations | This plan records the supersession of #30's segment-decomposition design and *why* (survey lines bypass the planner; XTE is analytic). |
| Improve incrementally | Leaves planner_server, controller, and the #28/#35 preemption flow untouched. |
| Human control and transparency | "Pilot handles holes" only holds if the deviation is visible → explicit #183 tie. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (ROS 2 conventions) | Yes | BT `SyncActionNode` + plugin registration + manifest per nav2/BehaviorTree.CPP conventions; ament test target. |
| ADR-0002 (worktree isolation) | Yes | Work in the `feature/issue-30` layer worktree; PR to `unh_marine_navigation`. |
| ADR-0013 (`progress.md` vocabulary) | Yes | Plan + subsequent work logged with standard entry types. |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `run_tasks.xml` `SurveyLine` ports | the installed copy under `core_ws/install` is build-generated — rebuild, don't hand-edit | Yes |
| Add a BT node | `bt_register_nodes.cpp` (regenerates manifest) + `nav2.btproj` palette + run_tasks.xml `<TreeNodesModel>` | Yes |
| Reshape the followed path | operator must see nominal vs deviated → `unh_echoboats_project11#183` | Tracked there (not this PR) |
| New costmap subscription in a BT node | blackboard `node` is externally spun (verified) → cached-subscription is sound; mutex snapshot across the two threads | Yes — resolved |

## Resolved Decisions

- **max_xte** — node param baseline + optional per-line task-field override; only
  the node side is wired in this PR (no task-message/CAMP plumbing).
- **Obstacle cost `g(cost)`** — raw graded cost, no baseline subtraction, no
  threshold; `INF` floor at inscribed/lethal. The `w_xte·d²` spring means only the
  obstacle-cost *gradient* (not a uniform offset) moves the path, so distant
  inflation can't bow the line. Tune the `w_xte : w_obs` ratio.
- **Costmap access** — cached subscription on the blackboard `node` (verified
  externally spun via the `BtActionServer` executor — `set_controller_speed.cpp`
  relies on it). Callback runs on the bt_navigator executor thread, distinct from
  the tick thread → `shared_ptr` pointer-swap snapshot under a mutex.
- **Pose frame** — de-risked by defaulting to TF-transform into the costmap frame
  (short-circuit only when `frame_id` already equals it), since `TaskInformation.poses`
  carry the publisher's frame. (Still worth confirming the actual frame during impl
  to know whether the transform is the hot path or the short-circuit is.)

## Open Questions

- None blocking. Remaining items are impl-time confirmations folded into Resolved
  Decisions above (actual pose frame; `w_xte:w_obs` and inflation `cost_scaling_factor`
  tuning values — set during sim).

## Estimated Scope

Single PR: one new BT node + its unit test, registration/manifest, and a one-line
`SurveyLine` wiring edit. Operator-visibility display is out of scope (#183).

## Implementation Notes

- **Pure core is `Station`/`makeLateralOffsets`/`resampleStations`/`solveCorridorOffsets`**
  (free functions in the header) so the DP is unit-tested with a hand-built cost
  matrix, no ROS. `tick()` only does costmap sampling, TF, robot-pose clipping,
  and reconstruction. 8 gtests pass (clear→on-line, blob→bounded re-anchored
  detour, wall→infeasible, lateral-rate limit, offset/resample helpers).
- **Costmap source**: subscribes to `nav2_msgs/Costmap` (`costmap_raw`, 0–254) so
  the inscribed/lethal `INF` floor and `NO_INFORMATION`→free mapping are exact;
  no `nav2_costmap_2d` dependency (raw index math). Added `nav2_msgs` + `nav_msgs`
  to CMake/package.xml.
- **Build gotcha (worktree + new BT node)**: `marine_nav_behavior_tree` runs a
  POST_BUILD `generate_..._nodes_xml` step that loads the freshly-built plugin
  `.so`. In a worktree the package is *overridden* (it also lives in the sourced
  main merged install), and the loader picks the stale main `.so` by SONAME —
  which lacks the new node's symbols → `undefined symbol: …AdjustPathForObstacles…`.
  Existing-package builds never hit this (the stale `.so` has all symbols); only a
  *new* node exposes it. Workaround used: prepend the package's own build dir to
  `LD_LIBRARY_PATH` before building so the generate step loads the fresh `.so`:
  `export LD_LIBRARY_PATH="$PWD/build/marine_nav_behavior_tree:$LD_LIBRARY_PATH"`.
  The same prepend is needed to run the unit test binary in a worktree.
