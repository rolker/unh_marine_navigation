---
issue: 8
---

# Issue #8 — Tests for marine_nav_bt_task_navigator (BT task navigator)

## Issue Review
**Status**: complete
**When**: 2026-05-26 21:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #8
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/8#issuecomment-4550419085
**Scope verdict**: needs-more-detail

### Actions
- [ ] Add `package.xml` `test_depend` + `CMakeLists.txt` `if(BUILD_TESTING)` test targets (currently lint-only) — consequence of adding tests (ADR-0008: `ament_cmake_gtest` / `launch_testing`, target Rolling).
- [ ] Re-frame the test strategy in the issue to match the plugin architecture: `TaskNavigator` is a `nav2_core::BehaviorTreeNavigator<RunTasks>` plugin hosted by `bt_navigator`; the action server is owned by the base, not this class. Contract is the protected overrides.
- [ ] Correct the downstream mock surface: `follow_path`, `hover`, `compute_path_through_poses`, `compute_sonar_coverage_path` (+ planner/controller selectors), not just "SmacPlanner, FollowPath".
- [ ] Decide (in plan-task) whether to split into (a) lightweight protected-method unit tests now vs. (b) deferred full-tree `launch_testing` integration test — open question, not decided in review.
- [ ] Sequence after #4 (`marine_nav_tasks`) and #7 (BT node plugins); have #9 (CI) pick up resulting tests.
- [ ] When writing preemption/completion tests, pin intended behavior for two latent code observations (or fix them here): hardcoded `"task_list"` blackboard key in `goalCompleted` (`src/task_navigator.cpp:141`); dead `terminatePendingGoal()` branch in `onPreempt`.
