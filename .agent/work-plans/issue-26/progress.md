---
issue: 26
---

# Issue #26 — Field import: per-task speed plumbing (2026-05-22)

## External Review
**Status**: complete
**When**: 2026-05-25 11:40
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 — 1 review, 9 inline comments, 6 valid (after collapsing duplicates), 1 false positive
**CI**: all-pass (`copilot-pull-request-reviewer` → success)

### Actions
- [x] Add `std::isfinite` check on the BT `speed` input in `SetControllerSpeed::tick()` before the `<= 0.0` guard; skip with throttled WARN if non-finite (Copilot #1).
- [x] In `CrabbingPathFollower::on_set_parameters_callback`, reject non-finite or non-positive `default_speed` by returning `SetParametersResult{successful=false, reason="..."}` (Copilot #9, paired defense-in-depth with the BT-side check).
- [x] Treat empty `target_node` input in `SetControllerSpeed::tick()` as an error — throw `BT::RuntimeError` to match the "missing" case (Copilot #2).
- [x] Add explicit `#include "rcl_interfaces/msg/set_parameters_result.hpp"` to `set_controller_speed.cpp` (Copilot #4).
- [x] Remove custom `main()` from `test_set_controller_speed_resolve.cpp`; rely on `gtest_main` linked by `ament_add_gtest()` (Copilot #5 + #8).
- [x] Add `<Action ID="SetControllerSpeed">` entry to the inline `<TreeNodesModel>` block in `run_tasks.xml` to match the sibling plugin entries — fixes both Copilot #6 and #7.
- [ ] (Optional) Reply to Copilot #3 (`last_pushed_speed_` update timing) explaining the false-positive rationale: resetting on failure would flood the unthrottled rejection WARN on persistent validator rejection AND introduce a thread race against the BT thread's non-atomic read.

All valid findings addressed in commit (pending). 5/5 gtest cases still pass; build clean; no new lint findings introduced.

## External Review (round 2)
**Status**: complete
**When**: 2026-05-25 13:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 — second Copilot review at `4a3bdf1`, 4 new inline findings (the 7 carried-forward comments are visually duplicated by GitHub but already fixed in the previous review-iteration).

### Actions
- [x] Reset `last_pushed_speed_` to the sentinel when `service_is_ready()` returns false (Copilot R2 #1). Different failure mode than the one I'd rejected as FP earlier: controller_server restart → service comes back at YAML default → BT was silently dedup-skipping the per-task speed. Clearing the sentinel on the not-ready branch lets the next post-recovery tick re-send.
- [x] Add `find_package(rclcpp)` + `find_package(rcl_interfaces)` + `ament_target_dependencies(... rclcpp rcl_interfaces)` for `${PROJECT_NAME}_bt_plugins` in `marine_nav_behavior_tree/CMakeLists.txt` (Copilot R2 #2).
- [x] Add `<depend>rclcpp</depend>` and `<depend>rcl_interfaces</depend>` to `marine_nav_behavior_tree/package.xml` (Copilot R2 #3).
- [x] Add `find_package(rcl_interfaces)` + `ament_target_dependencies(... rcl_interfaces)` + `<depend>rcl_interfaces</depend>` to `marine_nav_crabbing_path_follower` CMakeLists + package.xml (Copilot R2 #4).

Build clean on both packages; gtest 5/5 pass; no new lint findings.

## External Review (round 3)
**Status**: complete
**When**: 2026-05-25 13:50 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 at `1ac59f2`
**Reviews**: 2 new inline comments at this head; 2 valid, 0 false positives
**CI**: all-pass

### Actions
- [x] Add `std::isfinite + > 0.0` validation to the initial `default_speed` read in `CrabbingPathFollower::configure()`. On invalid YAML/launch input, log a WARN and fall back to the declared default (1.0 m/s). Closes the symmetry gap: param callback validates updates, but the initial config-time read currently doesn't (Copilot R3 #1).
- [x] Rewrite the misleading comment block at `set_controller_speed.cpp:114-117`. The completion callback doesn't read/write `last_pushed_speed_` — it only inspects SetParameters results and logs. The real thread-safety story: `last_pushed_speed_` is only touched in `tick()`, which runs single-threaded on the BT loop (Copilot R3 #2).

Build clean; gtest 5/5 still pass.

## External Review (round 4)
**Status**: complete
**When**: 2026-05-25 15:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 at `0bf4fae`
**Reviews**: 3 new inline comments at this head; 3 valid, 0 false positives
**CI**: all-pass

### Actions
- [x] After `desired_speed_.store(1.0)` fallback in `CrabbingPathFollower::configure()`, also call `node->set_parameter(rclcpp::Parameter(plugin_name_ + ".default_speed", 1.0))` so the param service reflects the effective speed. Wrap in try/catch in case `set_parameter` throws during shutdown. Observability fix: `ros2 param get` will otherwise keep reporting the original NaN/Inf/<=0 value while the controller actually runs at the fallback (Copilot R4 #1).
- [x] Reword the snapshot comment at `crabbing_path_follower.cpp:175-177`. `desired_speed_` is `std::atomic<double>` so `load()` is already tear-free; the real reason to snapshot is consistency across the cycle (avoid a mid-cycle update between the speed-limit math and the DEBUG log) (Copilot R4 #2).
- [x] Extend the `speed` port description in `SetControllerSpeed::providedPorts()` to mention that non-finite values (NaN/Inf) are also skipped with a throttled WARN. Keeps the generated `marine_nav_behavior_tree_nodes.xml` in sync with the actual `isfinite` check landed in R3 (Copilot R4 #3).

Build clean; gtest 5/5 pass; auto-generated nodes XML reflects the updated docstring; param-server write is in place with `RCLError` catch.

## External Review (round 5)
**Status**: complete
**When**: 2026-05-25 16:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 at `70a203f`
**Reviews**: 3 new inline comments at this head; 3 valid, 0 false positives
**CI**: all-pass

### Actions
- [x] Broaden the `set_parameter()` catch in `CrabbingPathFollower::configure()` from `rclcpp::exceptions::RCLError` to `std::exception`. R4's catch was too narrow — `set_parameter()` could throw other `rclcpp::exceptions::*` types or `std::exception` during teardown/future API changes, propagating out of configure() and crashing controller bring-up. Defeats the safety guard's intent (Copilot R5 #1).
- [x] Complete dep declarations on `marine_nav_crabbing_path_follower`: CMakeLists `ament_target_dependencies` includes `nav2_costmap_2d` and `nav2_util` but neither has a `find_package` call or `<depend>` entry. Add `find_package(nav2_costmap_2d REQUIRED)` + `find_package(nav2_util REQUIRED)` to CMakeLists; add `<depend>nav2_costmap_2d</depend>` + `<depend>nav2_util</depend>` to package.xml. Pre-existing gap exposed by R2's partial cleanup (Copilot R5 #2).
- [x] Complete dep declarations on `marine_nav_behavior_tree/package.xml`: 7 missing `<depend>` entries (`behaviortree_cpp`, `geometry_msgs`, `marine_nav_interfaces`, `nav2_behavior_tree`, `nav2_util`, `tf2_ros`, `std_msgs`). All 7 already have `find_package` and `ament_target_dependencies`; only the package.xml side is stale. Pre-existing gap (Copilot R5 #3).

Build clean; gtest 5/5 pass.

## External Review (round 6)
**Status**: complete
**When**: 2026-05-25 16:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 at `97cba25`
**Reviews**: 2 new inline comments at this head (same root cause); 1 valid (collapsed), 0 false positives
**CI**: all-pass

### Actions
- [x] Refactor `SetControllerSpeed` to follow the selected controller. Replaced `parameter_name` port with `controller_name` (default `"FollowPath"`) + `parameter_suffix` (default `"default_speed"`); constructed full path in `tick()`. Both XML insertions pass `controller_name="{selected_controller}"`. Inline `<TreeNodesModel>` entry and auto-generated `marine_nav_behavior_tree_nodes.xml` updated. Empty-string guards on both new ports throw `BT::RuntimeError` to avoid the malformed `".default_speed"` / `"FollowPath."` cases. Cache key remains the resolved target-node name (unchanged) (Copilot R6 #1+#2 collapsed).

Build clean; gtest 5/5 pass; generated nodes XML reflects the new port set.

## External Review (round 7)
**Status**: complete
**When**: 2026-05-25 17:15 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 at `f365ee0`
**Reviews**: 1 new inline comment at this head; 1 valid, 0 false positives
**CI**: all-pass

### Actions
- [x] Type-check the configure-time `default_speed` read in `CrabbingPathFollower::configure()`. Currently `node->get_parameter(default_speed_param).as_double()` throws `InvalidParameterTypeException` if the parameter was declared with a non-double type — common when YAML has `default_speed: 1` (no decimal, parses as integer) or CLI `default_speed:=1`. The throw escapes `configure()` and aborts controller bring-up — defeats the safety guard. Match the live-update callback's strictness (only accept `PARAMETER_DOUBLE`); on any other type, route through the existing invalid-value fallback path so the WARN + param-server write is shared. Consolidated single WARN covering both type and value (Copilot R7 #1).

Build clean; gtest 5/5 pass.

## External Review (round 8)
**Status**: complete
**When**: 2026-05-25 18:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 at `3d32def`
**Reviews**: 2 new inline comments at this head; 2 valid, 0 false positives
**CI**: all-pass

### Actions
- [x] Mirror R7's type check on the runtime callback side: in `CrabbingPathFollower`'s `add_on_set_parameters_callback`, when name matches but type ≠ `PARAMETER_DOUBLE`, set `result.successful=false` with a reason including the actual type. Refactored the loop body — early-`continue` for non-matching names, early-`return` with reject for wrong type, early-`return` with reject for invalid value (existing). Matches the R7 configure-side type check (Copilot R8 #1).
- [x] Rework log lines to use `"on %s set %s = %.3f"` separator. Applied to the failure-WARN in the completion callback and the DEBUG line at the bottom of `tick()`. Output now reads `"SetControllerSpeed: on /bizzy/controller_server set FollowPath.default_speed = 1.5"` instead of the previous confusing `"/bizzy/controller_server.FollowPath.default_speed"` triple-dot form (Copilot R8 #2).

Build clean; gtest 5/5 pass.

## Local Review (Post-PR)
**Status**: complete
**When**: 2026-05-25 18:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**PR**: #27 at `eda10c3`
**Mode**: post-PR
**Depth**: Deep (reason: 670 lines + 12 files + cross-package within unh_marine_navigation)
**Must-fix**: 1 | **Suggestions**: 3

### Findings
- [x] (must-fix) `params_cb_handle_` registered in `configure()` but not reset in `cleanup()` — nav2 lifecycle hygiene; defensive against rclcpp callback-list ordering edge cases — `crabbing_path_follower.cpp:164-167` (Copilot Adversarial). **Addressed**: added `params_cb_handle_.reset();` to `cleanup()` with a comment explaining the pairing with `configure()`.
- [ ] (suggestion) Per-pose timestamp override silently negates per-task speed when path poses have non-zero `header.stamp` — **deferred to #32** per user. The override is intentional design: planners that consider dynamic obstacles can encode timings, and the controller honors them; per-task / YAML speed applies only when timestamps are absent. Implementation worked in the field.
- [ ] (suggestion) Task-without-speed inherits prior task's speed — **deferred to #32** (folded into the same precedence-design issue).
- [ ] (suggestion) Fast controller restart can defeat the dedup-reset — **deferred to #32** (folded as the optional case 4).

Follow-up issue: [rolker/unh_marine_navigation#32](https://github.com/rolker/unh_marine_navigation/issues/32) covers all three deferred suggestions plus the related concern that `speed_limit_` should also clamp timestamp-derived speeds.
