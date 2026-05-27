# Plan: BT SkipUnknownTaskType catchall silently marks tracklines done on subtree FAILURE

## Issue

https://github.com/rolker/unh_marine_navigation/issues/25

## Context

`NavigatorSequence` (`run_tasks.xml:127-170`) dispatches via a `ReactiveFallback`
("SelectTaskReactiveFallback") whose children are the per-type task subtrees, each
gated by a first-child `ScriptCondition(current_task_type == '<type>')`, with a
trailing `SetTaskDone name="SkipUnknownTaskType"` catchall.

Two distinct failure shapes share one exit:
- **Unmatched type** — no subtree's `ScriptCondition` passes; fallback walks to the
  catchall and marks done. *Correct.*
- **Matched-but-failed** — e.g. `current_task_type == 'survey_line'` enters
  `SurveyLineTask`, then `FollowPath` ABORTs → the subtree returns FAILURE → the
  fallback walks past the remaining (type-mismatched) subtrees → lands on the **same**
  `SetTaskDone` catchall → the trackline is recorded clean-done despite never running.
  Forensically observed in a field deployment (issue body).

Findings that shape the approach (verified in this worktree):
- **The record-semantics surface already exists.** `TaskInformation.msg` carries
  `string status` ("YAML structure with task status information"); `Task::setStatus()`
  / `status()` exist in both C++ (`task.cpp:98`) and Python (`task.py:128`); and
  `bt_types.cpp:245` already serializes `status` onto the feedback/heartbeat JSON.
  **`setStatus` is never called anywhere today.** So "attempted-but-failed, distinct
  from done" needs **no `.msg` change and no downstream rebuild** — just write the
  existing field. (Revises the Decisions note's "likely needs a task-status addition
  in `marine_nav_interfaces`" — investigation shows the field is already there.)
- **No diagnostics infrastructure exists** (`grep -i diagnostic` → nothing). Since the
  status field already reaches the operator on the heartbeat, observability can ride on
  that path + an `RCLCPP_ERROR` log rather than standing up a `DiagnosticStatus`
  publisher (which would be net-new scope). See Open Questions.
- **The retry pattern is in-repo.** `RunSurveyAreaSubTasks` (`run_tasks.xml:198`) and
  `SurveyLineSetTask` (`:330`) already wrap line execution in
  `RetryUntilSuccessful num_attempts="3"`; mirror it at the top level (Decision #1).
- BT format is `BTCPP_format="4"` (per #21), so the stock `Switch` node is available.

Decisions locked with Roland (2026-05-26, see `progress.md`): retry ~3× then
skip + flag (not halt — preserves out-of-comms autonomy); record attempted-but-failed
distinctly from done; structurally distinct exits for unmatched-type vs matched-but-failed.

## Approach

Single PR (`feature/issue-25`).

1. **Restructure dispatch as a `Switch` on `current_task_type`.** Replace the
   `ReactiveFallback` + per-subtree `ScriptCondition` gates with a `Switch` whose cases
   are the known types (`hover`, `goto`, `survey_line`, `survey_line_set`, `survey_area`)
   and whose **default branch is the existing `SetTaskDone "SkipUnknownTaskType"`**.
   A matched case that FAILS no longer falls through to the default — that is the core
   structural fix and the contract #35 builds on. The per-type `ScriptCondition` first
   children become redundant (the `Switch` does the type match) and are removed.
2. **Wrap each matched case in `RetryUntilSuccessful num_attempts="3"`**, mirroring the
   two existing call sites. Transient `FollowPath` ABORTs get retried in place.
3. **New BT node `SetTaskFailed`** (sibling of `SetTaskDone`): on persistent failure it
   calls `task->setStatus({outcome: failed, reason: <code>, attempts: N})` **and**
   `task->setDone()`, then `RCLCPP_ERROR`s, returning SUCCESS so the mission advances.
   Wire it as the fallback after each case's retry block so an exhausted retry records
   *attempted-but-failed* (status set) rather than *clean done* (status empty) — the
   distinction the issue's coverage-record stake needs. Registered in
   `bt_register_nodes.cpp` alongside `SetTaskDone`.
4. **Observability via the existing status path** — the failed `status` already flows to
   the operator on the heartbeat (`bt_types.cpp:245`); the `RCLCPP_ERROR` in step 3 covers
   the log side. (DiagnosticStatus deferred — see Open Questions.)
5. **Minimal inline test (per decision: tests now, not waiting on #8).** Add a gtest for
   `SetTaskFailed` (asserts it sets `done` **and** writes a non-empty `status` with the
   failed marker; contrast with `SetTaskDone` leaving status empty). This adds gtest infra
   to `marine_nav_behavior_tree`. The *routing-level* regression (matched-but-failed does
   not reach the default catchall) is integration-shaped and is the higher-value test —
   see Open Questions for whether to add a focused XML-tree fixture now or ride #8.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | `NavigatorSequence`: `ReactiveFallback`→`Switch` on `current_task_type`; per-case `RetryUntilSuccessful` + `SetTaskFailed` fallback; default = existing `SetTaskDone`; drop now-redundant per-subtree `ScriptCondition` gates; update embedded `TreeNodesModel` for `SetTaskFailed` |
| `marine_nav_behavior_tree/src/plugins/action/set_task_failed.cpp` + `include/.../set_task_failed.h` | **New** node: `setStatus(failed marker)` + `setDone()` + `RCLCPP_ERROR` |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register `SetTaskFailed` |
| `marine_nav_behavior_tree/CMakeLists.txt` + new `test/` | Add node to plugin lib; add `ament_cmake_gtest` infra + `SetTaskFailed` test |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Failed line is recorded (status) + logged (ERROR) + surfaced on heartbeat — no longer silent. |
| A change includes its consequences | Reuses existing `status` field → no interface/downstream ripple; test lands with the node; survey-area / line-set levels evaluated (Open Questions). |
| Test what breaks | What breaks is the dispatch *routing*; node-level gtest is the floor, routing fixture flagged as the higher-value test. |
| Only what's needed | No new `.msg`, no diagnostics publisher unless chosen; `Switch` + one node + retry reuse. |
| Capture decisions | Rationale (retry-not-halt, reuse status field, defer DiagnosticStatus) recorded here + `progress.md` + PR body (no `docs/decisions/` in this repo). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Work on `feature/issue-25` layer worktree; PR into `jazzy`, no direct default-branch commits. |
| 0008 — ROS 2 conventions | Yes | Stock BT.CPP4 `Switch`/`RetryUntilSuccessful`; new node follows the `SetTaskDone` `SyncActionNode`/ports idiom; uses the existing `status` field rather than inventing a parallel channel. |

## Consequences

| If we change... | Also update... | In this PR? |
|---|---|---|
| `NavigatorSequence` dispatch shape | `SurveyLineSetTask` / `RunSurveyAreaSubTasks` share the silently-mark-done-on-exhausted-retry shape at their levels | Evaluate; fix in-scope if mechanical, else flag follow-up (Open Questions) |
| Write `TaskInformation.status` for the first time | Any operator-side consumer parsing the heartbeat `status` (cross-repo grep `mission_manager` / `rqt_*`) — confirm none assumes it stays empty | Audit before merge; no `.msg` change |
| Add `SetTaskFailed` node | Embedded `TreeNodesModel` in `run_tasks.xml` + generated `*_nodes.xml` | Yes |

## Open Questions

- [ ] **DiagnosticStatus vs status-field+log.** Decision #1 said "emit DiagnosticStatus
  ERROR." No diagnostics infra exists; the status field already reaches the operator.
  Recommend riding on status + `RCLCPP_ERROR` now and treating a `DiagnosticStatus`
  publisher as a separate observability issue. Confirm with Roland.
- [ ] **Routing-level regression test now, or ride #8?** Node-level gtest is included.
  The bug is in routing; a focused BT-XML fixture (stub subtrees forced to FAILURE,
  assert default catchall not reached) is the test that actually catches #25. Add it now
  (more than "minimal") or defer the integration test to #8's harness?
- [ ] **Mirror at survey-area / line-set levels?** Their `RetryUntilSuccessful` +
  `SetTaskDone` also marks done on exhausted retry. Fix all three here (consistent) or
  scope #25 to the top-level dispatch and file a follow-up?
- [ ] **#35 hand-off.** #35 depends only on "matched-but-failed is structurally distinct
  and not clean-marked-done." Confirm the `Switch` + `SetTaskFailed` shape satisfies its
  identity-gate re-entry before #35 finalizes its plan.

## Estimated Scope

Single PR in `unh_marine_navigation`. Small-to-moderate: one new `SyncActionNode`
(+gtest), a `Switch`-based rewrite of one `BehaviorTree`, retry reuse. **No interface
change** (existing `status` field) and **no downstream rebuild** — the main scope risk
in the recorded decisions is removed. Unblocks #35 once the dispatch shape is confirmed.
