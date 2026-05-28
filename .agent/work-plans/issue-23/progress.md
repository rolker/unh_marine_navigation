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

## Implementation
**Status**: complete
**When**: 2026-05-28 00:00 -04:00 (impl) → 2026-05-28 09:00 -04:00 (review-driven hardening)
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Branch**: `feature/issue-23`
**Commits**:
- `40b8f3f` — fix: zero outer `path.header.stamp` in SetPathFromTask + GetSubPath (initial fix + 16 GTest cases + plan amendment)
- `c8c8e1c` — style: apply uncrustify to new test files (whitespace; new test files only — pre-existing package-wide drift left untouched)
- `54c8c61` — fix: guard buildPath/buildSubPath against signed/unsigned wrap (round-2 hardening from adversarial review; 4 additional GTest cases)

**Tests**: 31/31 marine_nav_behavior_tree gtests pass (test_set_path_from_task 10/10, test_get_sub_path 10/10, existing 11/11). Pre-existing uncrustify drift on untouched files remains out-of-scope (see PR #37 for the same situation).

**Plan step 6 (bag/sim validation)**: GTest regression covers the failure class as a pure-function unit test (assertion matrix: outer stamp zero on non-empty result, outer frame_id preserved, per-pose stamps untouched, empty/single/inverted/very-negative ranges). The 2026-04-27 gabby bag replay was the additional field-event validation step; deferred — not blocking. The structural fix (zero-stamp = "latest" in tf2 lookups) is the load-bearing change and the unit tests cover its observable contract.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-28 09:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: `feature/issue-23` at `54c8c61`
**Mode**: pre-push
**Depth**: Standard (reason: BT plugin + Nav2/tf2 interaction; 9 files, ~500 lines, no security/cross-layer triggers)
**Must-fix**: 1 | **Suggestions**: 3

### Findings
- [x] (must-fix) Range math signed/unsigned wrap in `buildPath`/`buildSubPath` — very-negative end_index leaves end_index < 0 after normalization; cast to size_t wraps to a huge value, loop copies whole vector. Cross-model agreement (Claude Adversarial + Copilot Adversarial, independently). Addressed in `54c8c61` with explicit bounds guard + 4 new GTest cases. — `set_path_from_task.cpp:60-68`, `get_sub_path.cpp:42-50`
- [x] (suggestion) Zero-stamp comment "mirrors path_to_pose_vector.cpp's idiom" was misleading — same idiom but different field (outer Path header here vs per-pose stamps there). Reworded in `54c8c61`. — `set_path_from_task.cpp:78-80`, `get_sub_path.cpp:64-66`
- [x] (suggestion) `makeStalePath()` comment "outer (stale, what the bug propagated)" misattributed the bug source — the original code copied the first per-pose stamp (1000), not the input outer (999). Reworded in `54c8c61`. — `test_get_sub_path.cpp:27`
- [ ] (suggestion, declined) `task_bb.value()->message()` doesn't guard against a null shared_ptr in the blackboard (Copilot Adversarial, single-source). Declined: per the workspace's "don't validate for scenarios that can't happen" rule (`CLAUDE.md`), and no current call site puts null in `{current_task}`. The BT runtime would surface a segfault as a node failure rather than silently mask it.
