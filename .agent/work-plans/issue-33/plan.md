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
   - `onCycleUpdate`: **at the top**, transform the target into `local_frame_` **into a
     per-cycle local copy** (keeping `target_pose_` in its received frame) via `tf_`
     (frame-agnostic; identity when already in `local_frame_`, real transform if held in map).
     Wrap in try/catch on `tf2::TransformException` → return `TF_ERROR` (mirror the existing
     `getCurrentPose` failure path, `hover.cpp:64-67`); don't let it propagate. A local copy
     (not in-place) is required so a target held in a drifting frame keeps tracking that frame
     every cycle rather than being frozen at engagement — see Implementation Notes. The v4
     speed/steering logic below is **untouched**.
3. **New BT node `PredictStoppingPose`** (`BT::SyncActionNode`, ported from `1c5db5a^`):
   - Ports: input `deceleration` (double < 0), output `pose` (PoseStamped). Good port
     descriptions (they feed the auto-generated Groot nodes-XML — see step 7).
   - Reads `tf_buffer`, `odom_smoother`, `robot_frame`, and `global_frame` from the blackboard
     (`tf_buffer`/`odom_smoother` seeded by the nav2 base; `robot_frame`/`global_frame` by
     task_navigator). **No subscription** (BT node runs on the unspun client node).
   - Math (static, unit-tested helper `projectStoppingPose`): look up current pose
     (robot_frame → global_frame) via `tf_buffer` → position + orientation; get smoothed body
     twist from `odom_smoother.getTwist()`; rotate twist → world **using the orientation from
     that same tf lookup** (**the easy-to-miss gotcha**; full 2D velocity incl. crabbing);
     `stop = position + v̂·|v|²/(2·|decel|)`. Output PoseStamped in `global_frame`. Zero velocity
     or non-negative deceleration ⇒ pose == current.
   - Register in `bt_register_nodes.cpp`.
4. **`task_navigator.cpp` / `.h`** — declare `default_deceleration` param (default `-0.45`);
   seed `default_deceleration` **and** `global_frame` onto the blackboard in `configure()`
   (same pattern as the existing `robot_frame:41` seed).
5. **`hover_action.cpp` / `.h`** — add input port `target` (PoseStamped) → `goal_.target`.
6. **`run_tasks.xml` `HoverTask`** — insert
   `<PredictStoppingPose deceleration="{default_deceleration}" pose="{hover_target}"/>`
   **after** the transit-resolution Fallback (it runs on both paths — `AlwaysSuccess` stays as
   the no-transit marker), and pass `target="{hover_target}"` to `<Hover>`. Running it on every
   entry — rather than only the no-location branch — keeps `{hover_target}` fresh (no stale
   target leaking across hovers) and post-transit the boat is ~stopped so it ≈ the current pose.
   See Implementation Notes.
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
10. **Test** — the forward-vs-reverse choice is extracted into a pure, dependency-free helper
    `chooseApproachHeading` (`hover_heading.h`) and unit-tested (6 gtests covering the
    point_at_target bypass, the reverse threshold, the 90° tie, and directly-behind). The
    `onCycleUpdate` integration + v4-floor reconciliation is verified on-water (A/B the toggle).

## Files to Change

| File | Change | Commit |
|------|--------|--------|
| `marine_nav_behavior_tree/.../plugins/action/hover_action.{h,cpp}` | Rename ports `*_distance`→`*_radius`; zero-init; **(C2)** add `target` port | 1, 2 |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Port-attr rename; PredictStoppingPose after the HoverTask Fallback + `target` to Hover; embedded `TreeNodesModel` updated (Hover `target` port, PredictStoppingPose entry) | 1, 2 |
| `marine_nav_interfaces/action/Hover.action` | Add `geometry_msgs/PoseStamped target` **+ a comment documenting the empty-`frame_id`⇒hold-current sentinel** | 2 |
| `marine_nav_behaviors/{include/.../hover.h,src/hover.cpp}` | onRun sentinel; remove `deceleration_`; onCycleUpdate target→local_frame_; **(C3)** live `point_at_target` + reverse via `chooseApproachHeading` | 2, 3 |
| `marine_nav_behaviors/include/marine_nav_behaviors/hover_heading.h` + `test/` + CMakeLists/package.xml | **(C3, new)** pure `chooseApproachHeading` helper + gtest (adds gtest infra to the package) | 3 |
| `marine_nav_behavior_tree/.../plugins/action/predict_stopping_pose.{h,cpp}` | **New** BT node (reads `tf_buffer` + `odom_smoother` from blackboard; no subscription) | 2 |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register `PredictStoppingPose` | 2 |
| `marine_nav_bt_task_navigator/{.../task_navigator.h,src/task_navigator.cpp}` | `default_deceleration` param + blackboard seed | 2 |
| `marine_nav_bt_task_navigator/behavior_trees/nav2.btproj` | Left untouched — optional cosmetic; canonical model is the generated `*_nodes.xml` + the embedded `TreeNodesModel` (updated) | — |
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

## Implementation Notes

- **PredictStoppingPose runs on both HoverTask paths, not just the no-location branch.** The
  issue sketch replaced the dead `<AlwaysSuccess/>` (no-location branch) with the node. But
  `{hover_target}` is a persistent blackboard key: if only the no-location branch wrote it, a
  later *location* hover would read a stale projected point. Running PredictStoppingPose once
  after the transit-resolution Fallback writes `{hover_target}` fresh on every hover entry;
  post-transit the boat is ~stopped so the projection ≈ the current pose (the prior
  "hold post-transit pose" behavior), and an arriving-at-speed no-location hover gets the
  momentum projection. `HoverAction` also treats an unset/empty `target` as "hold current",
  so the path is safe even if `{hover_target}` is never written.
- **`onCycleUpdate` transforms the target into a per-cycle local copy, not in place.**
  Transforming `target_pose_` in place would freeze it in `local_frame_` after cycle 1 (the
  `frame_id == local_frame_` guard would skip subsequent transforms), defeating drift-tracking
  for a target held in a slowly-drifting frame (e.g. map). Keeping `target_pose_` in its
  received frame and transforming into a local copy each cycle preserves that tracking; the
  visualization marker therefore renders in the target's original frame (RViz resolves it).
- **Build-env gotcha (not a code issue):** building a single overlay package in the worktree
  picks up the **stale main `core_ws/install`** (it predates these source changes), shadowing
  the freshly-built interfaces/plugin libs — symptoms were an `undefined SetControllerSpeed::
  providedPorts` at the nodes-XML codegen step and `Hover::Goal has no member target`. Fixed
  by building `marine_nav_interfaces` first, sourcing the worktree `core_ws/install` so it
  precedes main, and prepending the plugin build dir to `LD_LIBRARY_PATH` for the codegen. The
  durable fix is a fresh main `core_ws` build; flagged to the user separately.
