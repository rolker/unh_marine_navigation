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
