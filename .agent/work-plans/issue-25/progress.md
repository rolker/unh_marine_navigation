---
issue: 25
---

# Issue #25 — BT SkipUnknownTaskType catchall silently marks tracklines done on subtree FAILURE

## Issue Review
**Status**: complete
**When**: 2026-05-26 21:19 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #25
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/25#issuecomment-4550412892
**Scope verdict**: well-scoped

### Actions
- [x] Surface the matched-but-failed reaction as a decision before implementing — **resolved 2026-05-26** (see Decisions below).
- [x] Decide regression-test placement explicitly — **resolved 2026-05-26**: defer to #8 (in flight, may land first); plan #25's test to slot into #8's harness rather than duplicate.
- [ ] Favor a stock BehaviorTree.CPP `Switch`/`IfThenElse` dispatch (or the in-repo `_failureIf`-precondition pattern from `RunSurveyAreaSubTasks`) so the unknown-type exit and matched-but-failed exit are structurally distinct (prevents a #18-style recurrence at a new level; keeps ADR-0008 alignment). — carried into plan-task.

## Decisions
**When**: 2026-05-26 21:39 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — decided with Roland

Settled the open questions from the Issue Review so plan-task can proceed:

1. **Matched-but-failed reaction → retry, then skip + flag.** When a matched task
   subtree fails on execution (e.g. `FollowPath` ABORT, not a type mismatch):
   retry the line (~3×, mirroring the existing `RetryUntilSuccessful num_attempts="3"`
   in `RunSurveyAreaSubTasks`); on persistent failure emit a `DiagnosticStatus`
   ERROR and advance to the next task. Chosen over halting the mission because it
   preserves autonomy out-of-sight/comms-down (the 2026-05-01 scenario) and does
   not couple the fix to `mission_manager`'s unverified on-failure (`done_hover`)
   behavior, which lives upstream — not in this repo.
2. **Record semantics — attempted-but-failed, not `setDone()`.** A skipped line
   must be recorded distinctly from a completed one, or the post-mission coverage
   record still can't tell them apart (the issue's actual stake). This likely needs
   a task-status addition in `marine_nav_interfaces`; confirm the downstream surface
   during plan-task (consequence: downstream packages + docs + tests).
3. **Structural fix** stays as in Action 3: distinct exits for unmatched-type vs
   matched-but-failed.

### Context for plan-task
- Execution path: `NavigatorSequence` is a Nav2 navigator plugin
  (`task_navigator.cpp`, `nav2_core::NavigatorBase`); the tree's final status
  becomes the navigate action result (`goalCompleted` → `BtStatus`). `done_hover`
  is upstream `mission_manager` reacting to that result, not in this repo.
- `SetTaskDone` calls `task->setDone()` (`set_task_done.cpp:38`) — no failure flag
  today; that's why an aborted line is recorded clean.
- Next step when work resumes: run `plan-task` for #25.

## Plan Authored
**Status**: complete
**When**: 2026-05-27 09:15 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-25/plan.md` at `d5f2647`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/37 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] DiagnosticStatus vs status-field+log: no diagnostics infra exists; status already reaches operator on heartbeat — recommend status + RCLCPP_ERROR now, defer a DiagnosticStatus publisher. Confirm with Roland.
- [ ] Routing-level regression test now (focused BT-XML fixture) or ride #8's harness? Node-level gtest for SetTaskFailed is included regardless.
- [ ] Mirror the fix at the survey-area / survey-line-set retry levels (same mark-done-on-exhausted-retry shape) or scope to top-level dispatch + follow-up?
- [ ] Confirm the Switch + SetTaskFailed shape satisfies #35's identity-gate re-entry before #35 finalizes.

### Plan-time scope revision
- Decisions note said record-semantics "likely needs a task-status addition in `marine_nav_interfaces`." Investigation: `TaskInformation.status` (free-form YAML) already exists, has C++/Python `setStatus`/`status`, and is already serialized onto the heartbeat (`bt_types.cpp:245`) — but is never written today. So **no `.msg` change / no downstream rebuild** is required; this removes the main scope risk in the recorded decisions.

## Open Questions Resolved + Merge with #35
**When**: 2026-05-27 09:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — decided with Roland

Resolved the four plan open questions and merged #25 with #35:

- **Q1 (observability)** → write a **structured** `TaskInformation.status`
  (`{outcome, reason, attempts}`) + `RCLCPP_ERROR`. It rides the heartbeat to **camp**,
  which renders mission status — so the existing field is the operator's eyes; no
  `DiagnosticStatus` publisher. Separate follow-up (camp repo): improve camp's
  mission-status display. No `.msg` change.
- **Q2 (test depth)** → node gtest **+** a minimal BT-XML **routing fixture**
  (matched-but-failed → `SetTaskFailed`; unmatched → catchall). No #8 dependency;
  full-tree integration still deferred to #8.
- **Q3 (scope across levels)** → mirror retry + `SetTaskFailed` at **all three** levels
  (`NavigatorSequence`, `RunSurveyAreaSubTasks`, `SurveyLineSetTask`). Nested levels gain
  skip-bad-line-and-continue instead of abandon-area-on-one-abort.
- **Q4 (#35 coupling)** → **merge #25 + #35** into this PR. The retry/`SetTaskFailed`
  machinery and #35's identity-gate FAILURE collide on the same signal (preemption vs
  execution failure); the seam belongs where the machinery lives. `feature/issue-25` /
  **PR #37** is canonical and closes both; PR #36 closed as superseded.

Plan rewritten to the combined scope (Parts A–D). Central risk: the Part C
preempt-vs-fail seam (a legitimate switch must not burn the retry budget or be flagged
`failed`) — pinned by the re-entry test fixture. `unh_marine_navigation#35` plan is
superseded by this combined plan.

## Plan Review
**Status**: complete
**When**: 2026-05-27 10:33 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — independent fresh-context sub-agent review (same agent identity, but no access to author reasoning)

**Plan**: `.agent/work-plans/issue-25/plan.md` at `ebb2da9`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/37
**Verdict**: changes-requested

Diagnosis, merge rationale, the Switch-vs-default structural fix, and principle/ADR
alignment are sound. But Part C (the central design artifact) was built on a wrong model
of the tree, and the status→operator path was misattributed. All findings below verified
against source before persisting.

### Findings
- [ ] (must-fix) Part C `SurveyLineTask` structure wrong: the survey `FollowPath` is two subtree levels down (`SurveyLineTask → TransitAndSurveyLine → SurveyLine`), behind a transit leg (`NavigateThroughWaypoints`). The plan's diagram places `FollowPath` directly under the identity gate — not implementable as drawn. — `run_tasks.xml:347,362,282`; `plan.md` Part C
- [ ] (must-fix) Identity gate can't halt a RUNNING `FollowPath` from inside `SurveyLineTask`'s memory `Sequence`; the reactive wrapper already exists inside `SurveyLine` (around `FollowPath`) — the gate must live there (and likely use `CancelAllNavigation` on re-latch). — `run_tasks.xml:282-302`
- [ ] (must-fix) Part C retry-budget concern under-characterized: real risks are no-halt/re-latch loop, transit-leg replay on re-entry, and `SetPathFromTask` (SyncAction) re-emitting the path every tick if placed above a reactive gate. Redesign against the real two-level nesting. — `set_path_from_task.cpp:60-71`
- [ ] (must-fix) Status→operator path misattributed: it is `task_navigator.cpp:116` (`taskMessages()` feedback) → `unh_marine_autonomy/.../camp_interface.py:listTasks` (`if len(task.status): ' status: ' + str(task.status)`), NOT `bt_types.cpp:245`. Writing `status` for the first time changes camp text TODAY — a cross-repo consequence the table reduces to "audit." (`.msg`-unchanged conclusion still holds.) — `camp_interface.py:94-95`
- [ ] (suggestion) "Structured status as a stable contract for camp" overstated — `camp_interface.py` renders `str(task.status)`; no consumer parses `{outcome,reason,attempts}` today.
- [ ] (suggestion) `Switch` is not reactive across ticks like the current `ReactiveFallback`; confirm the top-level `ReactiveSequence` re-tick of `UpdateCurrentTaskData` still drives re-selection on a mid-run type change.
- [ ] (suggestion) "Mirror at all three levels" is three *different* edits — nested levels use `_failureIf` + `RetryUntilSuccessful` already; not one repeated edit.
- [ ] (suggestion) gtest infra already exists (`test_set_controller_speed_resolve.cpp`) — "extend," not "add"; the re-entry fixture must model the real two-level `FollowPath` placement or it validates a tree that isn't shipped.
- [ ] (suggestion) `path_task_id`/`current_task_id` resolution across the three `SurveyLineTask` call sites (different id keys + `_autoremap`) is a correctness item, not just validation.
