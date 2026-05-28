# Plan: marine_nav: TF extrapolation error on multi-line survey goals (stale header.stamp on per-line goals)

## Issue

https://github.com/rolker/unh_marine_navigation/issues/23

## Context

Multi-line survey goals carry a stale `header.stamp` from task-upload time, so Nav2's `FollowPath` then does `lookupTransform(..., path.header.stamp)` with a time outside the TF buffer (~10 s default) → extrapolation error → goal aborts at the line transition. Real-world repro: `unh_echoboats_project11/docs/logs/2026/2026-04-27_dev_logs.md:421` — "weird turn onto 3rd line" on a live multi-line survey, bag at `~/data/logs/bizzyboat/` on gabby.

**Two producers, identical bug shape** — both inherit the path's outer header from the first pose's stale stamp. Tracing `run_tasks.xml` shows they reach Nav2 differently:

- `marine_nav_behavior_tree/src/plugins/action/set_path_from_task.cpp:68` — `path.header = path.poses.front().header;` (builds `{survey_path}` for SurveyLine). **Load-bearing**: `{survey_path}` is fed directly to `FollowPath` in the `SurveyLine` subtree — this is the actual TF-extrapolation repro path.
- `marine_nav_behavior_tree/src/plugins/action/get_sub_path.cpp:49` — `output_path.header = output_path.poses.front().header;` (builds `{transit_path}` for TransitAndSurveyLine at `run_tasks.xml:373`). **Defense-in-depth**: the `{transit_path}` flow goes through `FixPathOrientations` (pass-through for the outer header) → `PathToPoseVector` (drops the outer header — extracts a pose vector) → `ComputePathThroughPoses` (Nav2 plans a fresh path) → `FollowPath`, so the stale stamp never reaches Nav2's TF lookup directly. Still right to fix — same buggy shape, prevents a regression if downstream wiring changes.

`clear_path.cpp` is fine — it emits a default-constructed empty `Path()` (zero stamp). The codebase already has the idiomatic fix in `marine_nav_behavior_tree/src/plugins/action/path_to_pose_vector.cpp:33`: `p.header.stamp = builtin_interfaces::msg::Time();   // zero stamp = "latest" in TF lookups`. Mirror that.

**Per-pose stamps are load-bearing** — `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp:347–348` reads `p1.header.stamp` / `p2.header.stamp` as `segment_start_time` / `segment_end_time`, and `marine_nav_utilities/src/utilities.cpp:22` advances per-pose stamps by `distance/speed`. The fix touches only the outer `path.header.stamp` — per-pose stamps stay untouched.

**Consumer grep finding (Step 1)**: across `unh_marine_navigation`, `unh_marine_autonomy`, and `unh_echoboats_project11`, **no in-repo code reads the outer `path.header.stamp`** (only `fix_path_orientations.cpp` reads `path.header.frame_id`, and `bt_types.cpp` round-trips header for blackboard JSON). The only consumer is Nav2's external TF-lookup machinery, which honours zero-stamp as "latest." The "preserved on purpose?" caveat from the issue body is resolved — no other consumer depends on the legacy stamp value.

## Approach

1. **Verify the "preserved on purpose?" caveat.** *(Done during plan amendment — see Context.)* Grep across `unh_marine_navigation`, `unh_marine_autonomy`, `unh_echoboats_project11` returned **zero in-repo consumers** of the outer `path.header.stamp`. Only Nav2's external TF lookup reads it. Recorded in Context + reproduced in PR body.
2. **Pull the 2026-04-27 bag** from gabby `~/data/logs/bizzyboat/`; identify the line-3 transition window from the dev log. Canonical offline repro for pre/post validation. If pruned, fall back to a sim-based repro (publish a Path with old `header.stamp` and confirm the extrapolation error pre-fix → none post-fix).
3. **Apply the fix in both producers, via a static helper.** Extract a `static nav_msgs::msg::Path buildPath(...)` helper on each class — mirrors `PredictStoppingPose::projectStoppingPose` (the in-repo testability precedent). The helper performs the index-range copy and sets the outer header to `frame_id` only + zero stamp:
   ```cpp
   path.header.frame_id = path.poses.front().header.frame_id;
   path.header.stamp = builtin_interfaces::msg::Time();   // zero = "latest" in TF lookups (mirrors path_to_pose_vector.cpp:33; for #23)
   ```
   `tick()` becomes a thin BT-blackboard wrapper around the helper. Per-pose stamps in `path.poses` are unchanged.
4. **Add regression tests.** Two new GTest files mirroring `test_predict_stopping_pose.cpp` (static-helper style; no BT-tick fixture — and avoids depending on unmerged `test_dispatch_routing.cpp`/`test_set_task_failed.cpp` from open PR #37):
   - `marine_nav_behavior_tree/test/test_set_path_from_task.cpp` — exercises `SetPathFromTask::buildPath` with a stale-stamped pose vector.
   - `marine_nav_behavior_tree/test/test_get_sub_path.cpp` — exercises `GetSubPath::buildSubPath` with a stale-stamped input path.
   - Assertion matrix (both tests): outer `path.header.stamp` is zero; outer `frame_id` preserved (non-empty round-trip); per-pose stamps **not** zeroed (representative `sec`/`nanosec` survives input → output); empty range (`start > end` or empty input) returns a default-constructed `Path` with a zero header; single-pose range works; negative `end_index` (= "from end") normalizes correctly.
   - Update `marine_nav_behavior_tree/CMakeLists.txt` with two new `ament_add_gtest` blocks (link `${PROJECT_NAME}_bt_plugins`; `ament_target_dependencies`: `behaviortree_cpp`, `geometry_msgs`, `nav_msgs`, `rclcpp`; plus `marine_nav_tasks` for `test_set_path_from_task`).
5. **Build + run all `marine_nav_behavior_tree` tests.** New tests pass; existing tests still pass. (Pre-existing uncrustify 0.78.1 drift is package-wide and unrelated — out of scope.)
6. **Offline-validate against the bag (or sim repro).** Confirm the line-3 TF-extrapolation error no longer fires and that crabbing's segment timing on subsequent lines continues to work (per-pose stamps unchanged). Record bag-window evidence in PR body.
7. **Pre-push self-review via `/review-code`.** Address findings; flip the PR from `[PLAN]` draft to ready.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_behavior_tree/include/marine_nav_behavior_tree/plugins/action/set_path_from_task.h` | Add `static nav_msgs::msg::Path buildPath(const std::vector<geometry_msgs::msg::PoseStamped>&, int start, int end);` declaration. |
| `marine_nav_behavior_tree/src/plugins/action/set_path_from_task.cpp` | Implement `buildPath` (frame_id + zero-stamp idiom); simplify `tick()` to call it. |
| `marine_nav_behavior_tree/include/marine_nav_behavior_tree/plugins/action/get_sub_path.h` | Add `static nav_msgs::msg::Path buildSubPath(const nav_msgs::msg::Path&, int start, int end);` declaration. |
| `marine_nav_behavior_tree/src/plugins/action/get_sub_path.cpp` | Implement `buildSubPath`; simplify `tick()` to call it. |
| `marine_nav_behavior_tree/test/test_set_path_from_task.cpp` | New: static-helper test for `buildPath` per assertion matrix in step 4. |
| `marine_nav_behavior_tree/test/test_get_sub_path.cpp` | New: same shape for `buildSubPath`. |
| `marine_nav_behavior_tree/CMakeLists.txt` | Register the two new `ament_add_gtest` blocks (style: `test_predict_stopping_pose`). |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions | Step 1's grep finding (the "preserved on purpose?" question) is recorded in the PR body, not assumed away. |
| A change includes its consequences | Same-PR per-pose-stamp consumer verification (`crabbing_path_follower`, `utilities`); both buggy producers fixed, not just one; build + tests + bag-replay all in scope. |
| Only what's needed | ~3-line code change × 2 files, mirroring an in-repo precedent; no new abstractions. |
| Test what breaks | GTest regression covers the failure mode plus neighbouring invariants (per-pose stamps untouched, `frame_id` preserved); bag-replay covers the actual field repro. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Work in `layers/worktrees/issue-unh_marine_navigation-23/`. |
| 0008 — ROS 2 conventions | Yes (lightly) | `builtin_interfaces::msg::Time()` zero-stamp = "latest" is the standard tf2 idiom; mirrors in-repo precedent. |
| 0013 — `progress.md` vocabulary | Yes | Lifecycle: `## Issue Review` (done) → `## Plan Authored` (this skill) → `## Plan Review` → `## Local Review (Pre-Push)` → `## Integrated Review` → `## Implementation`. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Outer `path.header.stamp` emitted by `SetPathFromTask` / `GetSubPath` | Any consumer of the outer stamp other than Nav2 FollowPath | Yes — step 1 grep verifies none exist; step 6 bag-replay confirms downstream behaviour. |
| Per-pose `header.stamp` | (intentionally not changed) — `crabbing_path_follower:347–348`, `utilities.cpp:22` rely on them | Verified untouched by the fix. |
| Test target list | `marine_nav_behavior_tree/CMakeLists.txt` `ament_add_gtest` blocks | Yes — step 4. |

## Open Questions

- **Zero-stamp vs `node->now()`** — both work for the TF-lookup case; the in-repo precedent uses zero-stamp ("latest"). BT nodes don't carry an `rclcpp::Node` handle in this codebase pattern, so zero-stamp is also the simpler form. Going with the precedent unless review-plan objects.
- **2026-04-27 bag availability** — if pruned/rotated on gabby, sim-based repro is the fallback. Either way, validation is feasible.

## Estimated Scope

Single PR, ~S effort. Helper extraction + fix across 2 production files (~20 lines net) + 2 new test files + 2 CMakeLists `ament_add_gtest` blocks. Coordinate with:
- [#35 (PR #36)](https://github.com/rolker/unh_marine_navigation/pull/36) — same producer file (`set_path_from_task.cpp`); land #23 first since #35 is still plan-only.
- [#28 (PR #40)](https://github.com/rolker/unh_marine_navigation/pull/40) — competing root-cause hypothesis for the same 2026-04-27 / 2026-05-22 line-transition failures (`CancelFollowPath` zero-command window tripping FCU's 3 s GUIDED watchdog). Both can be true simultaneously; #23 does not foreclose #28.

## Out-of-scope sibling

`marine_nav_behavior_tree/src/plugins/action/set_polygon_from_task.cpp:63` has the same anti-pattern on `PolygonStamped` (`polygon.header = poses[i].header;` — likely costmap-area). Out of scope for this Path-focused issue; flag in PR body for a future ticket so it doesn't bite the costmap layer the same way.
