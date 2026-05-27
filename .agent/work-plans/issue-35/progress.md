---
issue: 35
---

# Issue #35 — Mission re-send mid-line doesn't take effect — BT latches FollowPath path on same-type task switch

## Issue Review
**Status**: complete
**When**: 2026-05-26 14:18 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #35
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/35#issuecomment-4550278746
**Scope verdict**: well-scoped

### Actions
- [ ] Make the preemption observable (log the `FollowPath` halt/restart) so reported task and executed path stay coupled.
- [ ] Capture the fix-direction rationale in the PR (no project ADR system exists).

### Decisions (review walkthrough with Roland, 2026-05-26)
Open questions from the review were resolved interactively before plan-task:

1. **Interrupt semantics — transit to N+1's start.** On a mid-line Execute of
   N+1, abandon N, navigate to N+1's start point, then survey from the start.
   Matches today's clear→resend behavior and gives cleanest coverage of N+1.
2. **Fix mechanism — identity-gated BT re-entry.** Latch the task id at entry;
   a `ReactiveSequence` condition fails when `current_task_id` changes, halting
   the running `FollowPath` and re-entering the `Sequence` fresh so
   `SetPathFromTask` + `TransitAndSurveyLine` recompute for N+1 (which yields
   decision #1's transit-to-start). Localized to `run_tasks.xml`. Goal-preempt/
   cancel at the nav2 level was rejected (still needs id-change detection and
   doesn't re-run the transit leg by itself).
3. **Regression test — deferred, reevaluate at implementation.** No BT-navigator
   test harness exists yet and an agent is actively working #8. Plan to ride on
   that harness once it lands rather than bootstrap a parallel one; revisit the
   test mechanism when implementing.
4. **Sequencing — #25 first, then #35 on top.** #25 restructures the same
   dispatch region (`ReactiveFallback` → likely `Switch`/`IfThenElse`, lines
   ~126–168) that #35 builds on, and changes how `SurveyLineTask` is reached.
   Let #25's dispatch shape settle, then attach #35's id-gating to the new
   structure. Complementary, not redundant: a `Switch` keyed on
   `current_task_type` still won't re-tick on survey_line→survey_line, so #35 is
   needed regardless. (#28 also edits this file — same-region awareness applies.)

## Plan Authored
**Status**: complete
**When**: 2026-05-26 14:42 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-35/plan.md` at `1f644a6`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/36 (`[PLAN]` prefix)
**Phases**: single PR, blocked on #25

### Open questions
- [ ] #25's final dispatch shape (Switch vs IfThenElse) sets where the identity condition attaches and confirms the re-entry FAILURE is contained — finalize plan after #25 lands.
- [ ] Survey-area reuse: does the identity gate also fix mid-line switches inside a survey area, and is that desired, or scope the new condition to the top-level path only?
- [ ] Test mechanism deferred to #8's harness shape; revisit at implementation.

## Redesign — in-place switch; un-merged from #25
**When**: 2026-05-27 11:25 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — decided with Roland

Superseded the identity-gate / transit-to-start approach above. Roland relaxed the
transit-to-start decision (review walkthrough, 2026-05-26) to **in-place switch**: on a
mid-line re-send the controller transitions onto the new line from the current position.

This collapses the fix. Verified facts: `SetPathFromTask` is a pure node; `UpdateCurrentTask`
re-selects `current_task` every tick; Nav2 `FollowPath` resends its goal on a path change
while RUNNING (`BtActionNode goal_updated_`/`send_new_goal` — `FollowPathAction.cpp` path
comparison pending a sim spike). So #35 = **make `{survey_path}` track `current_task`
reactively** (replace the one-time latch with a per-tick refresh at the shared `SurveyLine`
node). No identity gate, no `path_task_id`, no halt, no FAILURE → **decoupled from #25**.

Consequences of the un-merge:
- This issue is **standalone again** on `feature/issue-35`; PR #36 reopened (was closed as
  superseded). #25 reverts to PR #37, closing #25 only.
- Earlier "blocked on #25" and "ride #8's harness" framings drop: #35 no longer touches
  #25's dispatch, and gets its own re-entry fixture (modelling the real `SurveyLine`/
  `FollowPath` placement).
- Accepted trade: in-place switch may join the re-sent line partway (not from its start).

Plan rewritten to the in-place-switch scope.
