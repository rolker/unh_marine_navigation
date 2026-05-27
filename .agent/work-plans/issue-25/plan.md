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
   `FollowPath` (the last child of `SurveyLine`'s inner `ReactiveSequence`,
   `run_tasks.xml:286-302`) in a Nav2 `RecoveryNode num_retries=3` whose recovery branch
   is the stock **`Wait` BT action node** (a short dwell → behavior_server `wait`). Note:
   the recovery is invoked via a *BT action node* (`Wait`, or the repo's custom `Hover`
   node at `bt_register_nodes.cpp:52`), **not** the `behavior_server` plugin list — there
   is no stock `hover` BT node, so `Wait` is the default (Open Q1). Because `SurveyLine` is
   the shared unit, this is **one edit, not three** for the *retry* axis. Wrap scope =
   `FollowPath` only (not the whole `ReactiveSequence`), so the one-time
   `CancelAllNavigation` at `SurveyLine` entry (`:284`) is **not** re-run on retry.
   Caveats to honor (from review): (a) on retry the controller re-follows the **full**
   `{survey_path}` — `CrabbingPathFollower` resumes from the nearest path point, so
   re-coverage is minimal, but **validate in sim**; (b) recovery only rescues *transient*
   trips — a persistent stuck condition just re-trips the 10 s progress checker, exhausts
   the 3 retries, and falls through to `SetTaskFailed` (the intended terminal behavior).
3. **New `SetTaskFailed` node + record-failed exits — only on matched-but-failed.** When
   recovery is exhausted the subtree FAILs. `SetTaskFailed` (sibling of `SetTaskDone`)
   writes `status` (a short structured map, e.g. `{outcome: failed, reason: <error_code>,
   attempts: N}`; only `str()`-rendered by camp today — written structured for future use,
   not a consumed contract) + `setDone()` so the mission advances + `RCLCPP_ERROR`, and
   returns SUCCESS so a containing loop keeps running.
   - **Top level:** the `Switch` already separates unmatched (→ default catchall) from a
     matched case's FAILURE → route the matched FAILURE to `SetTaskFailed`.
   - **Nested loops (`RunSurveyAreaSubTasks` `:195`, `SurveyLineSetTask` `:330`) —
     must preserve the matched-vs-unmatched distinction (review must-fix).** These route
     sub-tasks with `_failureIf` inside a `ReactiveFallback`, where a *type-mismatch*
     legitimately FAILs a branch as the dispatch mechanism. A blanket failure-fallback
     here would mark a correctly-routed, type-mismatched sub-task as `failed` —
     re-conflating unmatched-vs-failed one level down. **Fix: convert each nested loop's
     dispatch to the same `Switch`-on-(sub-task-type) + default-catchall shape** as the top
     level, wrapping matched cases with the recovery+`SetTaskFailed`. A mismatched type then
     hits the default (not recorded failed); only a matched-but-failed sub-line reaches
     `SetTaskFailed` and the loop **continues** (skip-and-continue). Structurally parallel
     to the top-level fix, not a blanket fallback.
4. **Observability.** The failed `status` already reaches camp (fact above) + the
   `RCLCPP_ERROR`. No `DiagnosticStatus` publisher (none exists). A richer camp
   mission-status display is a separate `unh_marine_autonomy`/camp follow-up.
5. **Tests** (extend the existing `ament_cmake_gtest` infra — `test_set_controller_speed_resolve.cpp`
   already present): `SetTaskFailed` node gtest (sets `done` + writes non-empty
   `status`); a minimal BT-XML **routing fixture** asserting: (a) matched-but-failed →
   `SetTaskFailed`, not the catchall; (b) unmatched type → catchall; (c) a **mid-run
   `current_task_type` change** re-routes via the `Switch` (exercises Switch reactivity,
   not just a static value); (d) **nested loop**: a failed sub-line records `failed` AND
   the loop continues to the next line without abandoning the area (where Finding-1's bug
   lives). Full-tree integration deferred to #8's harness.

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

- [x] **Which recovery node** — resolved with Roland: **`Wait`** (stock BT node; simple
  dwell for transient progress-checker trips). The custom `Hover` node remains an
  alternative if station-keeping during recovery is later preferred.
- [ ] **`num_retries` count** for the `RecoveryNode` (default 3, mirroring the nested
  loops). Tunable; revisit if field behavior suggests otherwise.
- [ ] **Status second-consumer search** — before merge, grep the workspace for other
  consumers of the RunTasks feedback/result `tasks[].status` beyond `camp_interface.py`
  (e.g. `rqt_*`, logging) that might assume it stays empty.

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
