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
- `TaskNavigator : nav2_core::BehaviorTreeNavigator`; its `configure()` seeds only
  `robot_frame` onto the blackboard (`task_navigator.cpp:41`). Adding a `default_deceleration`
  blackboard var follows that exact one-line pattern.
- The legacy `PredictStoppingPose` (git `1c5db5a^`) used the **full `nav_msgs/Odometry`**
  (`odom.pose.pose` + body twist rotated by `odom.pose.pose.orientation`) and output in
  `odom.header.frame_id`. `nav2_util::OdomSmoother` exposes only twist (no pose), so the
  faithful port **subscribes to odom directly** — needs no tf and no smoother.
- All 3 behavior configs (ben, seafloor echoboat, vrx) set `behavior_server.local_frame:
  <tf_prefix>/odom`, so **Hover holds station in odom**. bizzy/izzy inherit the seafloor
  nav stack (their `.yaml` carry only bridge/viz config).
- Retiring `hover.deceleration` is **safe standalone**: rclcpp ignores param overrides for
  undeclared params (evidence: stale node-level overrides like `simulate_ahead_time` already
  coexist in the running behavior_server config). No load crash, no deploy lockstep.

Decisions locked with user: one PR, **3 atomic commits**; deceleration as a navigator-level
`default_deceleration` param → blackboard, retiring the vestigial `hover.deceleration`;
`point_at_target=false` ⇒ min-rotation, allow-reverse (LOIT_TYPE=0); fix the `hover_action`
port-name mismatch here (naming-consistent); harden `onCycleUpdate` to be frame-agnostic;
do the 3 config cleanups now.

## Approach

Single PR (`feature/issue-33`), **3 atomic commits**.

### Commit 1 — fix the hover_action port-name mismatch (latent UB)

`HoverAction::providedPorts()` declares `minimum_distance`/`maximum_distance` but
`initialize()` reads `minimum_radius`/`maximum_radius` (undeclared → reads **uninitialized
doubles** into the goal; masked only by `hover.cpp:52`'s `> 0.0` guard).
- Rename the BT ports to `minimum_radius`/`maximum_radius` (match `Hover.action` + behavior
  params); update the `run_tasks.xml` `<Hover>` attributes (`minimum_distance`→`minimum_radius`,
  etc.); zero-init the locals defensively.

### Commit 2 — Deliverable 1: restore stop-point projection (the overshoot fix)

1. **`Hover.action`** — add `geometry_msgs/PoseStamped target` to the goal. Empty `frame_id`
   ⇒ hold current pose (back-compat sentinel).
2. **`hover.cpp` / `hover.h`** —
   - `onRun`: `target_pose_ = goal->target.header.frame_id.empty() ? <getCurrentPose> : goal->target;`
   - Remove the `deceleration_` member + its declare + its read (retired).
   - `onCycleUpdate`: **at the top**, transform `target_pose_` into `local_frame_` via `tf_`
     (frame-agnostic; identity when both are odom, real transform if a platform holds in map).
     The v4 speed/steering logic below is **untouched**.
3. **New BT node `PredictStoppingPose`** (`BT::SyncActionNode`, faithful port of `1c5db5a^`):
   - Ports: input `deceleration` (double < 0), input `odom_topic` (default `"odom"`),
     output `pose` (PoseStamped).
   - Subscribes to odom (rclcpp node from blackboard `"node"`), caches latest `Odometry`.
   - Math: rotate body twist → world by `odom.pose.pose.orientation` (**the easy-to-miss
     gotcha**; full 2D velocity incl. crabbing), `stop = odom.pose.pose.position +
     v̂·|v|²/(2·|decel|)`; output PoseStamped in `odom.header.frame_id`. Zero velocity ⇒
     pose == current.
   - Register in `bt_register_nodes.cpp`.
4. **`task_navigator.cpp` / `.h`** — declare `default_deceleration` param (default `-0.45`);
   seed it onto the blackboard in `configure()` (same pattern as `robot_frame:41`).
5. **`hover_action.cpp` / `.h`** — add input port `target` (PoseStamped) → `goal_.target`.
6. **`run_tasks.xml` `HoverTask`** — replace `<AlwaysSuccess/>` with
   `<PredictStoppingPose deceleration="{default_deceleration}" pose="{hover_target}"/>`; pass
   `target="{hover_target}"` to `<Hover>`. The location branch (`GotoPose`) leaves
   `hover_target` empty so Hover holds the post-transit pose.
7. **`nav2.btproj`** — update the `PredictStoppingPose` palette entry to the new port
   signature (drop the dead `context` port).
8. **Test** — `PredictStoppingPose` gtest: heading 90° + body vx ⇒ world vy (rotation);
   crabbing (vy≠0); stop distance = v²/(2|decel|); zero-velocity ⇒ pose==current. (Closes #6's
   hover-node test gap for this node.)

### Commit 3 — Deliverable 2: live `point_at_target` (min-rotation, allow reverse)

9. **`hover.cpp` / `hover.h`** — declare `hover.point_at_target` bool (default `true`) in
   `onConfigure`; read it **live in `onCycleUpdate`** via `get_parameter` so
   `ros2 param set /<ns>/behavior_server hover.point_at_target false` takes effect next cycle.
   `false` ⇒ choose forward vs reverse approach by smaller |heading error| (target bearing vs
   bearing±π), steer toward the chosen heading, allow negative `linear.x` for the reverse case
   (ArduPilot `LOIT_TYPE=0`). `true` ⇒ current always-face behavior.
10. **Validation** — `onCycleUpdate` needs a behavior fixture; cover the forward-vs-reverse
    selection as a pure helper unit test if extractable, else on-water A/B (toggle live).

## Files to Change

| File | Change | Commit |
|------|--------|--------|
| `marine_nav_behavior_tree/.../plugins/action/hover_action.{h,cpp}` | Rename ports `*_distance`→`*_radius`; zero-init; **(C2)** add `target` port | 1, 2 |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Port-attr rename; HoverTask PredictStoppingPose branch + `target` to Hover | 1, 2 |
| `marine_nav_interfaces/action/Hover.action` | Add `geometry_msgs/PoseStamped target` | 2 |
| `marine_nav_behaviors/{include/.../hover.h,src/hover.cpp}` | onRun sentinel; remove `deceleration_`; onCycleUpdate target→local_frame_; **(C3)** live `point_at_target` + reverse | 2, 3 |
| `marine_nav_behavior_tree/.../plugins/action/predict_stopping_pose.{h,cpp}` | **New** BT node (odom subscription, faithful port) | 2 |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register `PredictStoppingPose` | 2 |
| `marine_nav_bt_task_navigator/{.../task_navigator.h,src/task_navigator.cpp}` | `default_deceleration` param + blackboard seed | 2 |
| `marine_nav_bt_task_navigator/behavior_trees/nav2.btproj` | Update palette entry signature | 2 |
| `marine_nav_behavior_tree/CMakeLists.txt` + new `test/` | Add node to plugin lib + gtest | 2 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | `.action` change + retiring `hover.deceleration` ⇒ config cleanups + `mission_manager` consumer check, all enumerated; tests land with the node. |
| Test what breaks | PredictStoppingPose body→world rotation is the gotcha, gets a deterministic unit test. |
| Human control & transparency | `point_at_target` live, tight-by-default (`true` = current behavior). |
| Improve incrementally | 3 scoped commits; faithful legacy port (no `SetPoseFromTask`/full rewrite); reuses existing infra. |
| Capture decisions | No ADR system in this repo; issue + PR body carry rationale (LOIT_TYPE analog, decel-as-capability, retire vestigial param). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | New optional `.action` field is back-compat (empty-frame sentinel); new node follows nav2 `SyncActionNode`/ports idioms; `default_deceleration` follows nav2 param idioms. |

## Consequences

| If we change... | Also update... | In this PR? |
|---|---|---|
| `Hover.action` (interface) | `hover.cpp` (server), `hover_action.cpp` (BT node), rebuild `marine_nav_interfaces` | Yes |
| Retire `hover.deceleration` | Remove the stale `hover.deceleration:` line from **3 behavior configs** (own-repo PRs, verify owners): `ben_project11`, `seafloor_echoboat_project11` (used by bizzy/izzy), `unh_marine_simulation` (vrx); add per-hull `default_deceleration` only if a boat differs from -0.45. **Safe standalone — not deploy-coupled** (stale override ignored). | No — 3 separate-repo PRs, **opened now**, ref #33 |
| Hover goal shape | `mission_manager` (`unh_marine_autonomy`) hover path compiles; `mission_manager/README.md` | Verify here; README in autonomy repo |

## Open Questions

All three planning forks resolved with the user (see Context). Remaining items are
**validation**, not design:
- Confirm the odom publisher stamps `header.frame_id == <tf_prefix>/odom` on each platform
  (the `onCycleUpdate` transform makes a mismatch non-fatal, but verify for correctness).
- On-water re-validation of the projection — depends on reverse/brake authority on vectored
  thrust (`unh_echoboats_project11#88`/`#86`); A/B the `point_at_target` toggle live.

## Estimated Scope

Single PR in `unh_marine_navigation` (3 commits) + 3 tiny config-cleanup PRs in 3 sibling
repos (opened now). On-water re-validation required before relying on it operationally.
