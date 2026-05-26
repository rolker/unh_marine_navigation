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
