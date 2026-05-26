# Plan: Hover overshoots station on engagement — restore stop-point projection + optional live point_at_target

## Issue

https://github.com/rolker/unh_marine_navigation/issues/33

## Context

`marine_nav_behaviors::Hover` is a position-only proportional controller: `onRun` sets
the hold target to `getCurrentPose()` and `onCycleUpdate` commands speed from
`current_range` only — no velocity/deceleration term. A boat arriving at ~1.5 m/s
overshoots a median 9 m. The `deceleration_` param is declared/read but **never used**
(`hover.cpp:26,39`). `PredictStoppingPose` exists only as a stale Groot palette stub in
`nav2.btproj` (referencing the dead `project11_navigation::Context`); `run_tasks.xml`
`HoverTask` has a dead `<AlwaysSuccess/>` no-location branch (`run_tasks.xml:60`).

Verified facts that shape the approach:
- `TaskNavigator : nav2_core::BehaviorTreeNavigator`; its `configure()` **receives**
  `odom_smoother` but drops it (never stored, never put on the blackboard —
  `task_navigator.cpp:12-44`). Only `robot_frame` is seeded (`:41`).
- The legacy `PredictStoppingPose` (git `1c5db5a^`) sourced odom+decel from `Context`,
  which no longer exists. Port must source velocity from `odom_smoother` + orientation
  from tf instead.
- No precedent for a BT node *consuming* a config param; seeding one onto the blackboard
  is a one-line extension of the `robot_frame` pattern. **Decision: navigator-level
  `default_deceleration` param → blackboard; retire the vestigial `hover.deceleration`.**

## Approach

Single PR (`feature/issue-33`), **two atomic commits**.

### Commit 1 — Deliverable 1: restore stop-point projection (the overshoot fix)

1. **`Hover.action`** — add `geometry_msgs/PoseStamped target` to the goal. Empty
   `frame_id` ⇒ hold current pose (back-compat sentinel).
2. **`hover.cpp` / `hover.h`** — `onRun`: `target_pose_ = goal->target.header.frame_id.empty()
   ? <getCurrentPose> : goal->target;`. Remove the `deceleration_` member, its
   `declare_parameter_if_not_declared`, and its `get_parameter` (retired). `onCycleUpdate`
   **untouched** (preserves the field-tuned v4 orbital-loop patch).
3. **New BT node `PredictStoppingPose`** (`BT::SyncActionNode`, ported from `1c5db5a^`):
   - Ports: input `deceleration` (double, m/s² < 0), output `pose` (PoseStamped).
   - Sources from blackboard: `odom_smoother` (velocity), `tf_buffer` + `global_frame` +
     `robot_frame` (current pose/orientation).
   - Math: rotate body-frame twist → world frame by current orientation (**the
     easy-to-miss gotcha**; use full 2D velocity incl. crabbing), `stop = pos +
     v̂·|v|²/(2·|decel|)`. Output PoseStamped in `global_frame`.
   - Register in `bt_register_nodes.cpp`.
4. **`task_navigator.cpp` / `.h`** — declare `default_deceleration` param (default `-0.45`);
   seed `odom_smoother`, `tf_buffer`, `global_frame`, and `default_deceleration` onto the
   blackboard in `configure()` (same pattern as `robot_frame:41`).
5. **`hover_action.cpp` / `.h`** — add an input port `target` (PoseStamped) and map it into
   `goal_.target`.
6. **`run_tasks.xml` `HoverTask`** — replace `<AlwaysSuccess/>` with
   `<PredictStoppingPose deceleration="{default_deceleration}" pose="{hover_target}"/>`; pass
   `target="{hover_target}"` to `<Hover>`. The location branch (`GotoPose`) leaves
   `hover_target` empty so Hover holds the post-transit pose.
7. **`nav2.btproj`** — update the `PredictStoppingPose` palette entry to the new port
   signature (drop the dead `context` port) so Groot stays honest.
8. **Test** — `PredictStoppingPose` gtest: heading 90° + body vx ⇒ world vy (rotation);
   crabbing (vy≠0); stop distance = v²/(2|decel|); zero-velocity ⇒ pose==current. (Overlaps #6.)

### Commit 2 — Deliverable 2: live `point_at_target` (min-rotation, allow reverse)

9. **`hover.cpp` / `hover.h`** — declare `hover.point_at_target` bool param (default `true`)
   in `onConfigure`; read it **live in `onCycleUpdate`** via `get_parameter` so
   `ros2 param set /<ns>/behavior_server hover.point_at_target false` takes effect next cycle.
   When `false`: pick forward vs reverse approach by smaller |heading error| (target bearing
   vs bearing±π); steer toward the chosen heading and allow negative `linear.x` for the
   reverse case (ArduPilot `LOIT_TYPE=0`). When `true`: current always-face behavior.
10. **Test/validation** — `onCycleUpdate` needs a behavior fixture; cover the
    forward-vs-reverse selection as a small pure helper unit test if extractable, else
    document as on-water A/B validation (toggle live).

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_interfaces/action/Hover.action` | Add `geometry_msgs/PoseStamped target` to goal |
| `marine_nav_behaviors/{include/.../hover.h,src/hover.cpp}` | onRun target sentinel; remove `deceleration_`; (C2) live `point_at_target` + reverse logic |
| `marine_nav_behavior_tree/.../plugins/action/predict_stopping_pose.{h,cpp}` | **New** BT node (ported) |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register `PredictStoppingPose` |
| `marine_nav_behavior_tree/.../plugins/action/hover_action.{h,cpp}` | Add `target` input port → goal |
| `marine_nav_bt_task_navigator/{include/.../task_navigator.h,src/task_navigator.cpp}` | `default_deceleration` param; seed `odom_smoother`/`tf_buffer`/`global_frame`/`default_deceleration` on blackboard |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | HoverTask: PredictStoppingPose branch + `target` to Hover |
| `marine_nav_bt_task_navigator/behavior_trees/nav2.btproj` | Update palette entry signature |
| `marine_nav_behavior_tree/CMakeLists.txt` | Add new node to plugin lib + gtest |
| `.../test/` (new gtest) | PredictStoppingPose math |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | `.action` change + `default_deceleration`/retiring `hover.deceleration` touch downstream configs + consumers — enumerated below; tests land with the node. |
| Test what breaks | PredictStoppingPose's body→world rotation is the gotcha and gets a deterministic unit test. |
| Human control & transparency | `point_at_target` is operator-toggleable live, tight-by-default (`true` = current behavior). |
| Improve incrementally | Two scoped commits; reuses existing `odom_smoother`/tf infra; surgical XML edit, no full legacy rewrite (no `SetPoseFromTask` restore). |
| Capture decisions | This repo has no ADR system; the issue + PR body carry the rationale (LOIT_TYPE analog, decel-as-capability, retire-vestigial-param). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | New optional `.action` field is back-compat (empty-frame sentinel); new BT node follows nav2 `SyncActionNode` + ports conventions; `default_deceleration` follows nav2 param idioms. |

## Consequences

| If we change... | Also update... | In this PR? |
|---|---|---|
| `Hover.action` (interface) | `hover.cpp` (server), `hover_action.cpp` (BT node), rebuild `marine_nav_interfaces` | Yes |
| Retire `hover.deceleration` (remove declaration) | **All 5 platform configs** must drop the `hover.deceleration:` line and add `<navigator>.default_deceleration` — else the behavior_server throws on an undeclared param at load. Separate PRs in: `ben_project11`, `seafloor_echoboat_project11`, `unh_marine_simulation` (vrx), `unh_echoboats_project11` (bizzy+izzy). **Must deploy in lockstep per platform.** | No — separate-repo PRs, coordinated |
| Hover goal shape | `mission_manager` (`unh_marine_autonomy`) hover path compiles; `mission_manager/README.md` | Verify in this PR; README in autonomy repo |

## Open Questions

- **Latent bug noticed (not in scope):** `hover_action.h` declares ports `minimum_distance`/`maximum_distance` but `initialize()` reads `minimum_radius`/`maximum_radius` — masked today because XML passes `0.0` and `onRun` ignores non-positive goal fields. Fix here while touching the file, or file a separate issue?
- **Frame consistency:** `PredictStoppingPose` output frame must match the frame `Hover::onCycleUpdate` compares against (`local_frame_`). Confirm `global_frame` seeded == behavior `local_frame_` on each platform.
- **Lockstep deploy:** retiring `hover.deceleration` couples this PR to the per-platform config PRs at deploy time (undeclared-param load error otherwise). Confirm sequencing is acceptable given the dev-freeze window, or keep the declaration one cycle as a deprecated no-op.

## Estimated Scope

Single PR in `unh_marine_navigation` (2 commits) + 4 coordinated config PRs in 4 sibling
repos. On-water re-validation required (relates to reverse/brake authority,
`unh_echoboats_project11#88`/`#86`); A/B `point_at_target` live on the water.
