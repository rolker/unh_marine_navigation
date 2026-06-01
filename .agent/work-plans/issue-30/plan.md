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
   costmap subscription (latest cached under a mutex) + a TF buffer.
2. **tick()** — pass-through fast path when `nominal_path` is empty or no costmap
   yet. Otherwise: resample nominal into stations that are **ahead of the boat and
   inside the costmap window**; clamp first & last active station to `d=0` (anchor
   ends → a true detour that returns to the line). For each station, offsets
   `d ∈ [−max_xte, +max_xte]` at `lateral_step`; world point `on_line(s) + d·left_normal(s)`
   (in `map_tide`); `node_cost = INF` if lethal else `w_xte·d² + w_obs·g(cost)`.
3. **DP over stations** (this is the whole "planner"): transition cost
   `w_smooth·(dᵢ−dₖ)² + w_temporal·(dᵢ−d_prev_tick(sᵢ))²`, pruning transitions
   that exceed `max_lateral_rate`. O(stations·offsets²), microseconds. Backtrack
   `d(s)`, weld reshaped stations onto the untouched tail, set headings along the
   path tangent, mirror `buildPath`'s zero-outer-stamp / keep-per-pose-stamp idiom
   (#23). Always returns `SUCCESS` — it's a transformer, not a gate.
4. **Blocked corridor → output nominal unchanged.** If min DP cost is `INF` (or no
   improvement beyond a republish deadband), emit the nominal path. This degrades
   to today's behaviour: Collision-Monitor reflex stop → `FollowPath` ABORT →
   existing `RecoveryNode` → `SetTaskFailed` → pilot. No new blockage logic.
5. **Wire into the BT** — in the `SurveyLine` `ReactiveSequence`, `SetPathFromTask`
   writes `{nominal_survey_path}`; insert `AdjustPathForObstacles` reading it and
   writing `{survey_path}`; `FollowPath` unchanged. The `ReactiveSequence` re-tick
   IS the monitor — a new obstacle reshapes the path next loop and `FollowPath`
   preempts on the changed path (the #35 `on_wait_for_result` mechanism). No
   separate watcher node.
6. **Register + manifest** — add the node to `bt_register_nodes.cpp` and the BT
   nodes manifest; declare it in the `.btproj`/groot palette if applicable.
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
| `marine_nav_behavior_tree/CMakeLists.txt` | Add source + test target |
| BT nodes manifest / `.btproj` | Declare the new node |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | One insert + one port rename in `SurveyLine` |
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
| Add a BT node | `bt_register_nodes.cpp` + manifest + `.btproj` palette | Yes |
| Reshape the followed path | operator must see nominal vs deviated → `unh_echoboats_project11#183` | Tracked there (not this PR) |
| New costmap subscription in a BT node | confirm the blackboard `node` is spun (Open Q3) | Yes — open question |

## Open Questions

- **max_xte source** — per-line task field set from survey spec, a node param, or
  reuse of the crab follower's existing XTE config? It is the one
  survey-meaningful number; do not guess.
- **Survey-line pose frame** — confirm it is `map_tide` (no TF) vs needs a
  transform into the costmap frame.
- **Spin of the blackboard node** — other BT nodes only *call services* in tick;
  confirm a cached costmap *subscription* is serviced by the navigator's executor,
  else spin a dedicated callback group or poll a costmap service.
- **Inflation-tail handling** — with `inflation_radius: 150 m`, obstacle cost is
  nonzero almost everywhere near a hazard. `g(cost)` must key off the
  inscribed/obstacle + sea_surface bands (or subtract a baseline) so the line does
  not bow away from distant inflation. (Design constraint, resolve during impl.)

## Estimated Scope

Single PR: one new BT node + its unit test, registration/manifest, and a one-line
`SurveyLine` wiring edit. Operator-visibility display is out of scope (#183).
