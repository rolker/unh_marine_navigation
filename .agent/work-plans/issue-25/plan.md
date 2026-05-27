# Plan: Contain matched-but-failed task dispatch (no silent mark-done)

**Closes #25.** Standalone again — #35 was un-merged after the design simplified (see
"History / un-merge" below). This plan covers only #25.

## Issue

https://github.com/rolker/unh_marine_navigation/issues/25 — SkipUnknownTaskType catchall
silently marks tracklines done on subtree FAILURE

## Context

`NavigatorSequence` (`run_tasks.xml:127-170`) dispatches via a `ReactiveFallback` whose
children are per-type subtrees gated by a first-child
`ScriptCondition(current_task_type == '<type>')`, with a trailing
`SetTaskDone "SkipUnknownTaskType"` catchall. **Unmatched-type and matched-but-failed
share that one exit**, so a `FollowPath` ABORT inside a matched `survey_line` falls
through to the catchall and the trackline is recorded clean-done. Field-observed.

Verified facts shaping the approach:
- **No recovery is wired today.** `run_tasks.xml` has no `RecoveryNode`; a `FollowPath`
  ABORT propagates straight up. The deployed `nav2_params.yaml`
  (`seafloor_echoboat_project11/echoboat_project11/config`) sets controller
  `failure_tolerance: 0.3` + a `SimpleProgressChecker` (no 0.5 m movement in 10 s →
  ABORT), so ABORTs are a real event. `behavior_server` *does* configure recoveries
  `[spin, backup, drive_on_heading, assisted_teleop, wait, hover]`, but nothing invokes
  them — and `spin`/`backup` are inappropriate for a boat; `hover`/`wait` are the
  marine-sensible ones.
- **The record surface exists.** `TaskInformation.msg` has `string status`;
  `Task::setStatus`/`status()` exist in C++ (`task.cpp:98`) and Python (`task.py:128`),
  and are **never written today**. So recording "failed" needs **no `.msg` change**.
- **The operator path for `status` is cross-repo.** `task_navigator.cpp:116` publishes
  `taskMessages()` (raw `TaskInformation`) as RunTasks feedback → consumed by
  `unh_marine_autonomy/.../mission_manager/camp_interface.py:listTasks`, which renders
  `if len(task.status): kv.value += ' status: ' + str(task.status)` into the camp
  heartbeat (stringified — not parsed). So writing `status` for the first time **changes
  camp's task line today**: `type: survey_line (done) status: <…>`. (NOT `bt_types.cpp`,
  which feeds Groot/JSON logging — corrected from an earlier draft.)
- BT is `BTCPP_format="4"` (#21) → stock `Switch` and `RecoveryNode` available.
- The survey `FollowPath` is **two subtree levels** below `SurveyLineTask`:
  `SurveyLineTask → TransitAndSurveyLine (run_tasks.xml:362) → SurveyLine (282)`, and the
  `FollowPath` sits inside `SurveyLine`'s own `ReactiveSequence` (`:286-302`), behind a
  transit leg (`NavigateThroughWaypoints`). `SurveyLine` is the shared survey-line
  execution unit, reused at all three dispatch sites.

Decision (with Roland): keep the retry-then-flag intent, implemented **properly** as a
Nav2 `RecoveryNode` invoking a marine recovery — not a bare `RetryUntilSuccessful`.

## Approach

Single PR on `feature/issue-25`.

1. **`Switch` dispatch.** Replace `NavigatorSequence`'s `ReactiveFallback` + per-subtree
   `ScriptCondition` gates with a `Switch` on `current_task_type`; the **default branch**
   is the existing `SetTaskDone "SkipUnknownTaskType"`. A matched case that FAILS no
   longer reaches the default — the core structural fix. Confirm the top-level
   `ReactiveSequence` still re-ticks `UpdateCurrentTaskData` so a type change mid-run is
   still observed (the existing clear→resend path relies on this).
2. **Marine recovery wrapper (centralized at `SurveyLine`).** Wrap the survey-line
   `FollowPath` (in `SurveyLine`, the shared unit) in a Nav2 `RecoveryNode num_retries=3`
   with a **marine recovery** (candidate: `Wait`, or the custom `hover` behavior — see
   Open Questions) as the recovery branch. A transient ABORT triggers recover-then-retry.
   Because `SurveyLine` is reused at all three call sites, this is **one edit, not three**
   — superseding the earlier "mirror at three levels" framing for the *retry* axis.
3. **New `SetTaskFailed` node + record-failed exits.** When recovery is exhausted the
   subtree FAILs. `SetTaskFailed` (sibling of `SetTaskDone`) writes
   `status` (a short structured map, e.g. `{outcome: failed, reason: <error_code>,
   attempts: N}`; rendered stringified by camp today) + `setDone()` so the mission
   advances + `RCLCPP_ERROR`. Wire it so a matched-but-failed case records-failed rather
   than clean-done, at the top-level `Switch` case **and** at the nested loops
   (`RunSurveyAreaSubTasks` `:198`, `SurveyLineSetTask` `:330`) so a failed sub-line is
   recorded and the area/set **continues** its remaining lines (skip-and-continue). This
   *record/advance* axis genuinely is three edits (different surrounding structures).
4. **Observability.** The failed `status` already reaches camp (fact above) + the
   `RCLCPP_ERROR`. No `DiagnosticStatus` publisher (none exists). A richer camp
   mission-status display is a separate `unh_marine_autonomy`/camp follow-up.
5. **Tests** (extend the existing `ament_cmake_gtest` infra — `test_set_controller_speed_resolve.cpp`
   already present): `SetTaskFailed` node gtest (sets `done` + writes non-empty
   `status`); a minimal BT-XML **routing fixture** (matched-but-failed → `SetTaskFailed`;
   unmatched → catchall; recovery exhaustion → `SetTaskFailed`, not clean-done). Full-tree
   integration deferred to #8's harness.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | `Switch` dispatch; `RecoveryNode` + marine recovery around `SurveyLine`'s `FollowPath`; `SetTaskFailed` on exhausted-failure at top `Switch` case + the two nested loops; embedded `TreeNodesModel` for `SetTaskFailed` |
| `marine_nav_behavior_tree/.../action/set_task_failed.{h,cpp}` | **New** node: structured `setStatus` + `setDone` + `RCLCPP_ERROR` |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register `SetTaskFailed` |
| `marine_nav_behavior_tree/CMakeLists.txt` + `test/` | Add node to plugin lib; extend gtest infra; node + routing fixtures |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control & transparency | Failures recovered, then recorded (`status` → camp) + logged. No silent mark-done. |
| A change includes its consequences | Reuses existing `status` (no `.msg`/rebuild ripple); cross-repo camp consumer named; recovery centralized at the shared `SurveyLine`; tests land with code. |
| Test what breaks | Routing + recovery-exhaustion exits get BT fixtures, not just a node test. |
| Only what's needed | One new node; stock `Switch`/`RecoveryNode`; recovery wrapper at one shared node. |
| Capture decisions | Recovery-not-bare-retry, marine recovery choice, camp path correction — here + `progress.md` + PR. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | `feature/issue-25`; PR into `jazzy`. |
| 0008 — ROS 2 conventions | Yes | Stock BT.CPP4 `Switch`/`RecoveryNode`; new node follows `SetTaskDone` idiom; reuses behavior_server recoveries + the `status` channel. |

## Consequences

| If we change... | Also update... | In this PR? |
|---|---|---|
| Write `TaskInformation.status` first time | `unh_marine_autonomy/.../camp_interface.py:listTasks` already renders it → camp text changes; verify no other consumer assumes empty | Audit; no `.msg` change. camp display polish = separate follow-up |
| `Switch` dispatch + `SetTaskFailed` at 3 record sites | `TreeNodesModel`/generated `*_nodes.xml`; confirm each level's success-path `SetTaskDone` still fires | Yes |
| Invoke a behavior_server recovery from the task BT | Confirm the chosen recovery (`wait`/`hover`) is registered + marine-appropriate | Yes |

## Open Questions

- [ ] **Which recovery behavior** in the `RecoveryNode` — `Wait` (simple dwell) or the
  custom `hover` (station-keep)? Lean `Wait` for transient progress-checker trips; `hover`
  if station-keeping during recovery is preferred. Confirm with Roland.
- [ ] **`num_retries` count** for the marine `RecoveryNode` (default 3, mirroring the
  nested loops). Tunable.

## Estimated Scope

Single PR, #25 only. Moderate: one new node + gtest, a `Switch` dispatch rewrite, a
`RecoveryNode` wrapper at the shared `SurveyLine`, `SetTaskFailed` at three record sites.
No interface change, no downstream rebuild.

## History / un-merge

This was briefly merged with #35 (PR #37 closed both; PR #36 closed). After revisiting the
mid-line-switch behavior, #35 moved to **in-place path switch** (reactive `{survey_path}`;
Nav2 `FollowPath` auto-resends on path change), which produces no FAILURE and **decouples
from #25** — so the two were un-merged. #35 is standalone again on `feature/issue-35` / PR
#36; this PR (#37) reverts to closing #25 only.
