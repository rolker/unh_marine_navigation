---
issue: 23
---

# Issue #23 — marine_nav: TF extrapolation error on multi-line survey goals (stale header.stamp on per-line goals)

## Issue Review
**Status**: complete
**When**: 2026-05-27 23:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #23
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/23#issuecomment-4560573280
**Scope verdict**: well-scoped

### Actions
- [ ] (Watch — Capture decisions) Resolve the "is the source stamp preserved for a reason?" question explicitly in plan.md (grep consumers of `path.header.stamp` across the workspace; record the finding).
- [ ] (Watch — A change includes its consequences) Verify in the same PR that the per-pose-stamp consumers (`marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp:347–348`, `marine_nav_utilities/src/utilities.cpp:22`) are unaffected — the proposed fix touches only the outer `path.header.stamp`.
- [ ] Pull the 2026-04-27 bag from gabby (`~/data/logs/bizzyboat/`); use the line-3 transition as the canonical offline repro for fix validation before deployment.
- [ ] Mirror the in-repo precedent for the fix idiom (`marine_nav_behavior_tree/src/plugins/action/path_to_pose_vector.cpp:33`, zero-stamp = "latest" in TF lookups) and the GTest fixture style from `test_dispatch_routing.cpp` / `test_set_task_failed.cpp` for the regression test.
- [ ] Coordinate with #35 (PR #36) — same file (`set_path_from_task.cpp`); land #23 first since #35 is still plan-only.

## Plan Authored
**Status**: complete
**When**: 2026-05-27 23:35 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-23/plan.md` at `6b61501`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/41 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] Zero-stamp ("latest" in TF) vs. `node->now()` — going with the in-repo precedent (zero-stamp at `path_to_pose_vector.cpp:33`) unless review-plan objects; BT nodes don't carry an `rclcpp::Node` handle in this codebase pattern anyway.
- [ ] 2026-04-27 bag availability on gabby — fall back to sim-based repro if pruned/rotated.

### Plan-derived scope notes (carried from Issue Review actions)
- Two producers, not one — `set_path_from_task.cpp:71` + `get_sub_path.cpp:49` (verified during planning).
- Step 1 grep + step 6 bag-replay address the "Watch" findings from Issue Review.
- Coordination with #35 reaffirmed in plan's Estimated Scope.
