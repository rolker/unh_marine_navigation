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
