# Plan: Mission re-send mid-line doesn't take effect — BT latches FollowPath path on same-type task switch

> **SUPERSEDED — merged into #25.** This work was merged with #25 (2026-05-27): the
> identity-gate FAILURE here and #25's new retry/`SetTaskFailed` machinery collide on the
> same FAILURE signal, so they share one dispatch rewrite. The combined plan (closing both
> #25 and #35) lives at `unh_marine_navigation` `feature/issue-25` → PR #37. Implement
> there, not on this branch. Kept for reference (the identity-gate design below feeds the
> combined plan's Part B/C).

## Issue

https://github.com/rolker/unh_marine_navigation/issues/35

## Context

`SurveyLineTask` (`run_tasks.xml:347-359`) is a `ReactiveSequence` whose only
re-entry gate is `ScriptCondition(current_task_type == 'survey_line')`. The
inner memory `Sequence` runs `SetPathFromTask` **once** (latching `{survey_path}`
from the task active at entry), then holds `TransitAndSurveyLine`'s `FollowPath`
RUNNING. On a mid-line Execute of N+1, `UpdateCurrentTaskData` advances the
*reported* task (heartbeat → N+1) every tick, but because N→N+1 is
`survey_line → survey_line` the gate never fails, the memory `Sequence` is never
re-ticked, and the boat keeps following N's path. Clear→resend works only because
clearing flips `current_task_type` away from `survey_line`, failing the gate and
halting `FollowPath`.

**Decisions taken in the review walkthrough** (see `progress.md` → Decisions):
transit to N+1's start; identity-gated BT re-entry; regression test deferred to
ride on #8's harness; **#25 lands first** and #35 builds on its restructured
dispatch.

## Approach

1. **Latch the path's source-task id in `SetPathFromTask`.** Add
   `OutputPort<std::string>("path_task_id")` emitting `task->message().id`
   alongside the existing path output. This records *which task* the latched
   `{survey_path}` came from — a C++ latch immune to blackboard-remap quirks.
   (Alternative considered: an XML `Script` node latching
   `path_task_id := current_task_id` — rejected as more fragile under
   `_autoremap`.)
2. **Gate re-entry on task identity, not just type.** Add a second condition to
   `SurveyLineTask`'s `ReactiveSequence`:
   `ScriptCondition(current_task_id == path_task_id)`, evaluated after the
   `survey_path` is latched. When `current_task_id` changes mid-line it fails →
   the `ReactiveSequence` halts the running memory `Sequence` (cancels
   `FollowPath`) → next tick re-enters fresh → `SetPathFromTask` recomputes the
   path **and re-latches `path_task_id`**, and `TransitAndSurveyLine` transits to
   the new line's start (decision: transit-to-start). Guard the first-entry case
   where `path_task_id` is still empty (condition must not fail before the path
   is set).
3. **Contain the resulting FAILURE — depends on #25.** A failed identity gate
   makes `SurveyLineTask` return FAILURE. In today's `NavigatorSequence`
   (`run_tasks.xml:128-170`) that falls through `SelectTaskReactiveFallback` to
   the `SkipUnknownTaskType` catchall, silently marking N+1 done — **exactly
   #25's bug**. Build step 2 on #25's restructured dispatch (`Switch`/`IfThenElse`
   on `current_task_type`), where a matched-but-failed subtree no longer reaches
   the blanket catchall. The contract #35 relies on: *matched-but-failed must not
   mark the task done.*
4. **Cover the survey-area reuse.** `SurveyLineTask` is also instantiated inside
   `RunSurveyAreaSubTasks` (`run_tasks.xml:198-206`), where the relevant id is
   `current_survey_area_task_id`, not `current_task_id`. Verify the identity gate
   resolves to the correct id under that call site's remapping (it already gates
   with `_failureIf` on type) — wire the comparison so it is correct in both
   contexts, or scope the new condition so the survey-area path is unaffected.
5. **Make the preemption observable.** Log the `FollowPath` halt/restart (e.g. an
   INFO on identity-gate failure) so reported task and executed path stay coupled
   — addresses the transparency aspect of the defect.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_behavior_tree/src/plugins/action/set_path_from_task.cpp` | Add `path_task_id` output port; emit `task->message().id` |
| `marine_nav_behavior_tree/include/.../set_path_from_task.h` | (if ports declared in header) reflect new port |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Add identity `ScriptCondition` to `SurveyLineTask`; wire `path_task_id`; first-entry guard; log node. Built on #25's restructured dispatch. |
| `.../README.md` (if it documents BT task flow) | Update to describe identity-gated re-entry |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Fix re-couples reported task to executed path; halt/restart is logged (step 5), not silent. |
| A change includes its consequences | Regression test deferred by decision to ride on #8; README updated if it documents BT flow; survey-area reuse covered (step 4). |
| Test what breaks | Target failure mode is the mid-line same-type interrupt — the test (via #8 harness) asserts followed path becomes N+1's. |
| Only what's needed / Improve incrementally | Minimal: one output port + one condition + a guard; no dispatch rewrite (that's #25). |
| Capture decisions | Fix-direction rationale recorded here + in `progress.md`; restated in the PR (no project ADR system). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Work on `feature/issue-35` layer worktree; PR into `jazzy`, no direct commits. |
| 0008 — ROS 2 conventions | Yes | BT.CPP4 semantics explicit (memory `Sequence` halt on `ReactiveSequence` gate failure); follows existing node/port idioms. |
| 0001 — Adopt ADRs | Watch | No `docs/decisions/` in this repo; capture rationale in PR. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `SetPathFromTask` ports | Every `run_tasks.xml` call site of `SetPathFromTask`; node docs/tests | Yes — audit call sites; tests via #8 |
| `SurveyLineTask` subtree body | Both call sites (top-level + `RunSurveyAreaSubTasks`) | Yes — step 4 |
| BT behavior on interrupt | README BT-flow docs | Yes — if present |

## Open Questions

- **#25 dispatch shape** — final structure (Switch vs IfThenElse) sets where the
  identity condition attaches and confirms FAILURE is contained. Plan finalizes
  after #25 lands.
- **Survey-area scope** — does the identity gate also fix mid-line switches
  inside a survey area, and is that desired, or should the new condition be
  scoped to the top-level path only? (step 4)
- **Test mechanism** — defer to #8's harness shape; revisit at implementation.

## Estimated Scope

Single PR, **blocked on #25**. Small once unblocked: one C++ output port, one BT
condition + first-entry guard + log, doc touch-up. Regression test rides on #8.
