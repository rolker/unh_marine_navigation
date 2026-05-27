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
  blackboard var follows that exact one-line pattern. The nav2 **base**
  `BehaviorTreeNavigator::on_configure` already seeds `tf_buffer` **and** `odom_smoother`
  onto the blackboard (`behavior_tree_navigator.hpp:223,226`) before the derived
  `configure()` runs — so both are available to BT nodes without extra seeding.
- The legacy `PredictStoppingPose` (git `1c5db5a^`) used the full `nav_msgs/Odometry`
  (`odom.pose.pose` + body twist rotated by `odom.pose.pose.orientation`). `OdomSmoother`
  exposes only twist (no pose) — but a BT `SyncActionNode` **cannot subscribe** on its own:
  the blackboard `"node"` is the bt_action_server's client node, which no main executor
  spins (nav2's `BtActionNode` works around this with a private `SingleThreadedExecutor` it
  spins inside `tick()` — `bt_action_node.hpp:55-58,266`). So the node reads the **smoothed
  twist from `odom_smoother`** and the **pose+orientation from `tf_buffer`** (both on the
  blackboard), rotates body→world, and projects. No subscription, no spin problem.
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
   - `onCycleUpdate`: **at the top**, transform `target_pose_` **in place** into `local_frame_`
     via `tf_` (frame-agnostic; identity when both are odom, real transform if a platform holds
     in map). Wrap in try/catch on `tf2::TransformException` → return `TF_ERROR` (mirror the
     existing `getCurrentPose` failure path, `hover.cpp:64-67`); don't let it propagate.
     In-place keeps `publish_visualization`'s marker frame (`hover.cpp:181,202`) consistent.
     The v4 speed/steering logic below is **untouched**.
3. **New BT node `PredictStoppingPose`** (`BT::SyncActionNode`, ported from `1c5db5a^`):
   - Ports: input `deceleration` (double < 0), output `pose` (PoseStamped). Good port
     descriptions (they feed the auto-generated Groot nodes-XML — see step 7).
   - Reads `tf_buffer`, `odom_smoother`, and `robot_frame` from the blackboard (all seeded by
     the nav2 base / task_navigator). **No subscription** (BT node runs on the unspun client
     node).
   - Math: look up current pose (robot_frame → odom) via `tf_buffer` → position + orientation;
     get smoothed body twist from `odom_smoother.getTwist()`; rotate twist → world **using the
     orientation from that same tf lookup** (**the easy-to-miss gotcha**; full 2D velocity incl.
     crabbing); `stop = position + v̂·|v|²/(2·|decel|)`. Output PoseStamped in the lookup frame
     (odom). Zero velocity ⇒ pose == current.
   - Register in `bt_register_nodes.cpp`.
4. **`task_navigator.cpp` / `.h`** — declare `default_deceleration` param (default `-0.45`);
   seed it onto the blackboard in `configure()` (same pattern as `robot_frame:41`).
5. **`hover_action.cpp` / `.h`** — add input port `target` (PoseStamped) → `goal_.target`.
6. **`run_tasks.xml` `HoverTask`** — replace `<AlwaysSuccess/>` with
   `<PredictStoppingPose deceleration="{default_deceleration}" pose="{hover_target}"/>`; pass
   `target="{hover_target}"` to `<Hover>`. The location branch (`GotoPose`) leaves
   `hover_target` empty so Hover holds the post-transit pose.
7. **Groot model** — the canonical node model is the **auto-generated**
   `marine_nav_behavior_tree_nodes.xml` (`CMakeLists.txt:151-153` POST_BUILD), driven by the
   new node's `providedPorts()` — so good port descriptions there are sufficient. The
   `nav2.btproj` palette (in `marine_nav_bt_task_navigator`) is the stale hand-maintained
   file; updating its `PredictStoppingPose` entry is **optional cosmetic cleanup**, not the
   source of truth.
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
   - **Reconcile with the v4 floor:** the speed floors at `hover.cpp:140-153` use
     `std::max(current_target_speed, <positive>)`, which would clobber a negative (reverse)
     `linear.x` back to positive and defeat reverse mode. The reverse branch must bypass or
     sign-mirror those `std::max` floors. This is the most likely place D2 silently fights
     itself — handle it explicitly.
10. **Validation** — `onCycleUpdate` needs a behavior fixture; cover the forward-vs-reverse
    selection as a pure helper unit test if extractable, else on-water A/B (toggle live).

## Files to Change

| File | Change | Commit |
|------|--------|--------|
| `marine_nav_behavior_tree/.../plugins/action/hover_action.{h,cpp}` | Rename ports `*_distance`→`*_radius`; zero-init; **(C2)** add `target` port | 1, 2 |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Port-attr rename; HoverTask PredictStoppingPose branch + `target` to Hover | 1, 2 |
| `marine_nav_interfaces/action/Hover.action` | Add `geometry_msgs/PoseStamped target` **+ a comment documenting the empty-`frame_id`⇒hold-current sentinel** | 2 |
| `marine_nav_behaviors/{include/.../hover.h,src/hover.cpp}` | onRun sentinel; remove `deceleration_`; onCycleUpdate target→local_frame_; **(C3)** live `point_at_target` + reverse | 2, 3 |
| `marine_nav_behavior_tree/.../plugins/action/predict_stopping_pose.{h,cpp}` | **New** BT node (reads `tf_buffer` + `odom_smoother` from blackboard; no subscription) | 2 |
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
| Hover goal shape | `mission_manager` (`unh_marine_autonomy`) builds a `TaskInformation(type='hover')` in Python — it does **not** construct the `Hover.action` goal (the BT `HoverAction` node does), so the new field can't break it. Its `README.md` is ROS1-era stale (`actionlib_msgs`, SMACH) — too stale for a token edit | No code change; defer README to its own doc issue or minimal Hover-line correction |

## Open Questions

All three planning forks resolved with the user (see Context). Plan-review (PR #34) found
the original "subscribe to odom" data path broken (unspun BT client node) and the v4-floor /
reverse-sign interaction — both folded in above. Remaining items are **validation**, not design:
- The tf-based pose lookup uses the same tf tree Hover already relies on, so the earlier
  "odom publisher stamps header.frame_id" question is moot (no direct odom subscription).
- On-water re-validation of the projection — depends on reverse/brake authority on vectored
  thrust (`unh_echoboats_project11#88`/`#86`); A/B the `point_at_target` toggle live.

## Estimated Scope

Single PR in `unh_marine_navigation` (3 commits) + 3 tiny config-cleanup PRs in 3 sibling
repos (opened now). On-water re-validation required before relying on it operationally.
