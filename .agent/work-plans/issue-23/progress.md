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

## Plan Review
**Status**: complete
**When**: 2026-05-27 23:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))  <!-- performed by a fresh-context general-purpose sub-agent for independence; not the in-context plan author -->

**Plan**: `.agent/work-plans/issue-23/plan.md` at `6b61501`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/41
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix, narrative) Causal chain holds only for `SetPathFromTask` → `{survey_path}` (directly to `FollowPath`); `GetSubPath` → `{transit_path}` is dropped at `PathToPoseVector`/`ComputePathThroughPoses` so its stale stamp never reaches Nav2 `FollowPath`. Fixing `GetSubPath` is defense-in-depth, not load-bearing. Revise plan Context + PR body accordingly. — `plan.md` Context
- [ ] (must-fix, minor) Line-number drift: `set_path_from_task.cpp:71` is actually line `68` in the worktree; the `setOutput` is at 71. Update plan or describe by content. — `plan.md:11`
- [ ] (suggestion) Test-fixture references depend on PR #37 (still OPEN, not merged); `test_dispatch_routing.cpp` / `test_set_task_failed.cpp` do not yet exist on `jazzy`. Pick a strategy: wait for #37, rebase #23 onto #37, or write tests fresh without copying that pattern. — `plan.md` Approach step 4
- [ ] (suggestion) Step 1's grep finding (consumers of outer `path.header.stamp`) should be recorded in the PR body, not assumed — satisfies the issue-review "Capture decisions" Watch note. — PR body
- [ ] (suggestion) Add `marine_nav_tasks` (and likely `nav_msgs`) to the new test target's `ament_target_dependencies`; existing test blocks don't include this. — `plan.md` Files to Change
- [ ] (suggestion) Spell out the test-assertion matrix: empty path (guard skips buggy line), single-pose path, `frame_id` round-trip with non-empty value. — `plan.md` Approach step 4
- [ ] (suggestion) Acknowledge divergence between the plan's test ("outer stamp is zero") and the issue body's literal AC ("goal stamps advance per-line") — the zero-stamp form is correct for the chosen fix; note in PR body. — PR body
- [ ] (suggestion) Add coordination note for **#28 (PR #40)** alongside the existing #35 (PR #36) entry — both reference the same field events with different mechanisms (TF extrapolation vs `CancelFollowPath` FCU watchdog). — `plan.md` Estimated Scope
- [ ] (informational) `set_polygon_from_task.cpp:63` has the same anti-pattern on `PolygonStamped`; out-of-scope sibling, flag in PR body for a future ticket. — out of plan
- [ ] (suggestion) Add a sentence in plan Context noting `FixPathOrientations` is a pass-through for the outer header — structural evidence the producer-side fix is sufficient. — `plan.md` Context
