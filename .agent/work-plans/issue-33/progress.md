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
