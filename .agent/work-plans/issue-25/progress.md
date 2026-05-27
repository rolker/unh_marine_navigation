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
