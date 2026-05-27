---
issue: 33
---

# Issue #33 — Hover overshoots station on engagement — restore stop-point projection (PredictStoppingPose) + optional live point_at_target

## Issue Review
**Status**: complete
**When**: 2026-05-26 14:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #33
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/33#issuecomment-4549709879
**Scope verdict**: well-scoped

Central source claims verified against the tree: `deceleration_` declared/read but
unused; `Hover.action` has no target field; `PredictStoppingPose`/`SetPoseFromTask`
unregistered (palette-only); dead `<AlwaysSuccess/>` at `run_tasks.xml:60`;
`point_at_target` is net-new.

### Actions
- [ ] Update the `HoverAction` BT node (`hover_action.cpp`/`.h`) to expose `target` / `point_at_target` input ports and map them into the goal — implied by the issue's XML but not called out explicitly.
- [ ] Register `PredictStoppingPose` (and `SetPoseFromTask` if taking the legacy `Fallback` wiring) in `bt_register_nodes.cpp`.
- [ ] Add a deterministic `PredictStoppingPose` unit test guarding the body→world velocity rotation + full-2D crabbing term (overlaps #6).
- [ ] Split Deliverable 1 (overshoot bug fix) and Deliverable 2 (point_at_target enhancement) into separate atomic commits in one PR.
- [ ] Update cross-repo consumers/configs: per-platform `nav2_params.yaml` (ben_project11, seafloor_echoboat_project11, vrx sim, bizzy/izzy in unh_echoboats_project11 — own PRs per repo); verify `mission_manager` (unh_marine_autonomy) goal path still compiles; update `mission_manager/README.md` if goal shape changes.
- [ ] Surface the `point_at_target=false` heading behavior (forward/reverse minimal-rotation; allow reverse?) to the operator before finalizing — UX choice.
- [ ] State the on-water validation + dev-freeze plan in the PR; don't merge unvalidated by default. Validation depends on reverse/brake authority (unh_echoboats_project11#88/#86).

## Plan Authored
**Status**: complete
**When**: 2026-05-26 19:56 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-33/plan.md` (revised after open-question triage)
**PR**: https://github.com/rolker/unh_marine_navigation/pull/34 (`[PLAN]` prefix)
**Phases**: single PR, **3 commits** (C1 port-name fix, C2 D1 stop-point projection, C3 D2 live point_at_target)

Decisions locked with user: one PR / 3 commits; deceleration via navigator-level
`default_deceleration` param seeded onto the blackboard (robot_frame precedent), retire
the vestigial `hover.deceleration`; `point_at_target=false` ⇒ min-rotation allow-reverse
(LOIT_TYPE=0). PredictStoppingPose **subscribes to odom directly** (faithful `1c5db5a^`
port — `OdomSmoother` lacks pose), outputs in `odom.header.frame_id`; no tf/smoother needed.

### Open questions
All three planning forks resolved with the user:
- [x] Port-name mismatch → **fix here, naming-consistent** (rename BT ports `*_distance`→`*_radius`, own commit C1).
- [x] Frame consistency → **harden `onCycleUpdate`** to transform target into `local_frame_` (frame-agnostic); node outputs odom frame by construction.
- [x] Retiring `hover.deceleration` → confirmed **safe standalone** (rclcpp ignores stale undeclared overrides; no crash, no lockstep). 3 config-cleanup PRs (ben/seafloor/vrx) opened **now**, ref #33.

Remaining = validation only: confirm odom `header.frame_id` per platform; on-water re-validation (reverse/brake authority, `unh_echoboats_project11#88`/`#86`); A/B `point_at_target` live.

## Plan Review
**Status**: complete
**When**: 2026-05-26 20:18 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) (fresh-context sub-agent review; author orchestrated)

**Plan**: `.agent/work-plans/issue-33/plan.md` at `f2d2b3a` (reviewed state)
**PR**: https://github.com/rolker/unh_marine_navigation/pull/34
**Verdict**: changes-requested → addressed inline (see follow-up plan commit)

Independent review run via fresh-context sub-agent; all structural claims re-verified by
the author against nav2 source before acting.

### Findings
- [x] (must-fix) PredictStoppingPose "subscribe to odom" data path is dead — BT client node is unspun (`bt_action_node.hpp:55-58,266`). Fix: read `tf_buffer` (pose) + `odom_smoother` (twist) from the blackboard, both seeded by the nav2 base (`behavior_tree_navigator.hpp:223,226`). Reverses the Q2 subscription sub-decision (frame-hardening part of Q2 stands). Corrects my earlier wrong "odom_smoother is dropped" finding — the base class seeds it.
- [x] (must-fix) Groot model is auto-generated (`marine_nav_behavior_tree_nodes.xml`, CMakeLists POST_BUILD) from `providedPorts()`; `nav2.btproj` is stale hand-file → optional cosmetic. Plan updated.
- [x] (must-fix) v4 speed floor (`hover.cpp:140-153`, positive `std::max`) clobbers D2's reverse `linear.x<0`; reverse branch must bypass/sign-mirror. Added to commit 3.
- [x] (suggestion) onCycleUpdate transform: in-place + try/catch→TF_ERROR; rotation quaternion from same tf lookup as pose; empty-frame sentinel comment lands in `Hover.action`; mission_manager doesn't build the goal (no break) and its README is ROS1-stale → defer. All folded in.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 09:14 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-33 at `f3b3bfd`
**Mode**: pre-push
**Depth**: Deep (reason: safety-critical vehicle-motion control logic + new BT node + cross-package Hover.action change)
**Must-fix**: 2 | **Suggestions**: 3
**Specialists**: Static Analysis, Governance, Plan Drift, Claude Adversarial (Deep), Copilot Adversarial (Deep, v1.0.54). Static analysis: only repo house-style deltas (uniform across changed + untouched lines), nothing new. Build/tests NOT compiled/run in this review.

### Findings
- [x] (must-fix) Frame regression — PredictStoppingPose looks up the pose in the navigator `global_frame` (`<tf_prefix>/map`; seafloor/ben `nav2_params.yaml:4`) while Hover controls in `local_frame` (`<tf_prefix>/odom`, `:293`/`:217`). Hover now newly depends on the `map→base_link` TF; on a map outage / pre-localization engagement, `getCurrentPose` fails → node returns FAILURE → as a bare child of `SequenceWithMemory` (no Fallback) it aborts the whole HoverTask, where the old odom-only Hover kept running. Cross-confirmed by Claude + Copilot adversarial. Fix: project in `local_frame_`/odom (also realigns with this plan's locked Q2 "outputs odom frame by construction"). — `predict_stopping_pose.cpp:90-101`, `run_tasks.xml:71`
- [x] (must-fix) Stale-timestamp transform during sustained hover — because `global_frame(map) != local_frame(odom)`, onCycleUpdate transforms the engagement-stamped target every cycle using its fixed original stamp; hovers run 20–40 s, so once that stamp ages out of the TF buffer (~10 s) the lookup throws ExtrapolationException → TF_ERROR → Hover fails mid-station-keeping. Same root cause/fix as above (project in odom ⇒ per-cycle transform skipped), or transform at `Time(0)`. — `hover.cpp:96-111`
- [ ] (suggestion) Empty-`frame_id` "hold current pose" sentinel is unreachable in the wired tree — PredictStoppingPose always populates `{hover_target}` before `<Hover>`. Valid for direct action clients/defensive code, but if must-fix #1 adds a graceful PredictStoppingPose fallback it should route to this sentinel. — `hover.cpp:65`, `run_tasks.xml:71-80`
- [x] (suggestion) No test covers the combined reverse path (`point_at_target=false` + reverse-selected + inside-deadband negative term + v4 floor + `drive_sign`); pieces are individually correct but sign-subtle. Add a focused test or document the on-water A/B. — `hover.cpp:129-206` → resolved in `bf4a43d`: extracted the v4 speed logic to a pure `computeHoverSpeed()` (behavior-identical) and added `test_hover_speed.cpp` (11/11) covering range bands, taper, turn floor, and the forward/reverse composition — incl. the reverse-inside-deadband case that pushes the boat away from the point.
- [x] (suggestion) The 3 sibling-repo config-cleanup PRs (ben/seafloor/vrx) to retire `hover.deceleration`, which the plan states were "opened now", do not exist; stale `hover.deceleration:` lines remain as undeclared overrides (safe only under lenient param handling). Open them or confirm the param is absent from deployed YAML. — opened: seafloor `seafloor_echoboat_project11#31` (issue #30), ben `ben_project11#25` (issue #24), sim `unh_marine_simulation#61` (issue #60). Each removes the one `deceleration: -0.45` line under `behavior_server.hover`; safe in either merge order (param unused by current code too).

### Div-by-zero guard — `3ca7e77`
Re-review of the fixes surfaced a pre-existing latent bug: `computeHoverSpeed` divides by the
radius span and by `minimum_radius`, so `minimum_radius >= maximum_radius` (clearest: `max==min`
→ turn-floor `0/0`) or a negative `minimum_radius` put NaN/inf on `cmd_vel.linear.x`. Guarded the
pure helper to return 0.0 (hold) on invalid/NaN radii; `Hover::onRun` warns once per engagement.
Tests cover max==min / max<min / negative min (finite zero, never NaN). Built clean; 13/13 gtests.

### Review fixes applied — `82a5546`
Both must-fix items resolved: PredictStoppingPose now projects in the OdomSmoother's
odom frame (`getTwistStamped().header.frame_id`, = behavior `local_frame`), falling back
to `global_frame` only before odom arrives; Hover resolves a differently-framed target at
the latest transform (stamp 0). Removes the map dependency (no more HoverTask abort on a
map outage) and the stale-stamp expiry. Built clean; functional gtests pass
(`projectStoppingPose` 6/6, `chooseApproachHeading` 6/6). Suggestions 3–5 left open.

## Integrated Review
**Status**: complete
**When**: 2026-05-27 12:11 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #34 at `9e39bd2`
**Sources**: 2 — Copilot review @ `0e13583` (code current; head differs only by a progress note), Local Review (Pre-Push) timeline
**Cross-source confirmations**: 0
**CI**: none reported (no checks on head)

### Findings
- [ ] (valid, Copilot) `marine_nav_behaviors` uses `tf2`/`tf2_geometry_msgs` directly (`tf_->transform`, `tf2::durationFromSec`/`TransformException`/`getYaw`, the transform include) but declares neither in `package.xml`/`CMakeLists.txt` — relies on transitive nav2 exports. Add `find_package` + `ament_target_dependencies(marine_nav_hover_behavior … tf2 tf2_geometry_msgs tf2_ros)` + `<depend>` entries (mirror the BT package). — `marine_nav_behaviors/{CMakeLists.txt,package.xml}`, `src/hover.cpp:5`
- [ ] (valid, Copilot) PredictStoppingPose output-port doc still says "in the navigator's global frame", but `tick()` now outputs the odom frame from `getTwistStamped()` (falls back to global only when empty) — stale after the frame fix. Update the `OutputPort` description. — `predict_stopping_pose.cpp:33`

### False positives
(none — both Copilot comments are accurate; neither cross-confirmed, both single-source)
