# Plan: Mission re-send mid-line — make the followed path track the current task

**Closes #35.** Standalone (decoupled from #25 — see "Design history").

## Issue

https://github.com/rolker/unh_marine_navigation/issues/35 — Mission re-send mid-line
doesn't take effect; BT latches the `FollowPath` path on a same-type task switch.

## Context

`SurveyLineTask` (`run_tasks.xml:347-360`) is
`ReactiveSequence[ ScriptCondition(type=='survey_line'), Sequence[ SetPathFromTask, TransitAndSurveyLine, SetTaskDone ] ]`.
The inner memory `Sequence` runs `SetPathFromTask` **once** (latching `{survey_path}` from
the task active at entry), then holds `TransitAndSurveyLine`'s `FollowPath` RUNNING. On a
mid-line Execute of N+1, `UpdateCurrentTaskData` advances the *reported* task every tick
(heartbeat → N+1), but because N→N+1 is `survey_line → survey_line` the
`ScriptCondition(type)` still passes, the memory `Sequence` is **not** re-entered, and the
boat keeps following N's latched path. Clear→resend works only because clearing flips the
*type*, failing the gate. The defect is the **latch + type-only re-entry**.

Verified facts shaping the approach:
- **`SetPathFromTask` is a pure `SyncActionNode`** (`set_path_from_task.cpp`): reads
  `{current_task}`, slices `poses[start..end]`, `setOutput("path")`. No side effects —
  safe to re-run every tick.
- **`UpdateCurrentTask` re-selects the highest-priority not-done task every tick** and
  re-emits `current_task`/`current_task_id`/`current_task_type` (`update_current_task.cpp`).
  So `{current_task}` already tracks the operator's re-send within one tick.
- **Nav2 `FollowPath` accepts a new path while RUNNING without halting.** `BtActionNode`
  re-sends the goal when `goal_updated_` is set, via `send_new_goal()`, staying RUNNING
  (`bt_action_node.hpp:236-250`); `FollowPathAction` overrides `on_wait_for_result` (called
  each RUNNING tick). **Load-bearing, not yet fully confirmed:** that override must compare
  the input `path` port and set `goal_updated_` on change. I could not read
  `follow_path_action.cpp` (not shipped in `/opt/ros`) — Step 1 below is a sim spike to
  confirm. If it does **not** auto-resend, the fallback (also simple) is an explicit
  detect-change → cancel → resend in the BT; either way no #25 coupling.
- `SurveyLine` (`run_tasks.xml:282-302`) wraps the survey `FollowPath` in a
  `ReactiveSequence` that re-ticks each tick — a natural home for a per-tick path refresh.
- `SurveyLine` is the **shared** survey-line execution unit (reused under
  `RunSurveyAreaSubTasks` and `SurveyLineSetTask`), so a fix there covers all call sites,
  each using its own autoremapped `current_task`.

Decision (with Roland): **in-place switch** — on re-send the controller transitions onto
the new line from the current position (relaxing the earlier "transit-to-start" choice).
Accepted trade: the re-sent line may be joined partway rather than from its start.

## Approach

Single PR on `feature/issue-35`.

1. **Spike (first): confirm Nav2 `FollowPath` auto-resends on a path-input change.** In
   sim, start a survey line, change `current_task` to a different line mid-follow, and
   observe whether the controller picks up the new path without an ABORT/halt. This decides
   step 2's mechanism. (10-min spike; do not build the plan past this without the answer.)
2. **Make `{survey_path}` track `current_task` reactively.** Replace the one-time
   `SetPathFromTask` latch with a per-tick refresh so `{survey_path}` always equals the
   current task's path. Candidate placement: a reactive `SetPathFromTask` co-located with
   `FollowPath` in `SurveyLine`'s existing `ReactiveSequence` (it already re-ticks). When
   the operator re-sends, `{survey_path}` updates and `FollowPath` re-sends its goal in
   place. **Exact wiring confirmed against the live tree during implementation** (per the
   Part C lesson — no asserted structure here). If Step 1 shows no auto-resend, instead add
   a tick-level "path changed?" check that cancels + re-enters `FollowPath`.
   - The initial transit leg (`NavigateThroughWaypoints` in `TransitAndSurveyLine`) runs
     once on first entry and is **not** re-run on a switch — that is the in-place behavior.
3. **Verify the three call sites.** `SurveyLine`/`SurveyLineTask` are instantiated at the
   top level, inside `RunSurveyAreaSubTasks`, and inside `SurveyLineSetTask`, each with its
   own autoremapped current-task key. Confirm the reactive refresh derives the path from
   the correct task at each site (it reads the autoremapped `{current_task}`).
4. **Observability.** Log (INFO) when the followed path is re-sent due to a task change, so
   the reported task and executed path stay visibly coupled.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Make `{survey_path}` reactive to `current_task` at `SurveyLine` (replace the one-time latch with a per-tick refresh); INFO log on path re-send; embedded `TreeNodesModel` if node usage changes |
| (spike-dependent) `marine_nav_behavior_tree/...` | Only if Step 1 shows no auto-resend: a small "path-changed → cancel/resend" helper/condition node |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control & transparency | Followed path re-couples to the reported task; re-send is logged. |
| A change includes its consequences | Reuses pure existing nodes; three call sites verified; spike de-risks the load-bearing Nav2 assumption before building on it. |
| Test what breaks | The break is "path doesn't update mid-follow" — the re-entry fixture asserts `{survey_path}` and the followed goal track a mid-line task change. |
| Only what's needed | No identity gate, no `path_task_id`, no new dispatch machinery — just make an existing input live. |
| Capture decisions | In-place-vs-transit-to-start rationale here + `progress.md` + PR. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | `feature/issue-35`; PR into `jazzy`. |
| 0008 — ROS 2 conventions | Yes | Uses Nav2 `FollowPath`'s documented dynamic-path-update behavior; no bespoke mechanism unless the spike forces the (still-idiomatic) cancel/resend fallback. |

## Consequences

| If we change... | Also update... | In this PR? |
|---|---|---|
| `{survey_path}` becomes per-tick reactive | Confirm `GetSubPath`/transit reads stay correct (transit uses index 0 once); confirm re-deriving each tick is side-effect-free (it is) | Yes |
| In-place switch (no transit-to-start) | Operator expectation: re-sent line may be joined partway. Note in PR; revisit only if coverage gaps observed | Documented, not coded |
| Behavior depends on Nav2 `FollowPath` resend | Step-1 spike result; fallback path if absent | Yes |

## Open Questions

- [ ] Step-1 spike outcome (Nav2 auto-resend yes/no) — sets the step-2 mechanism.
- [ ] In-place switch may under-cover the re-sent line's start — acceptable for now per the
  decision; flag if field use shows it matters.

## Estimated Scope

Single PR, #35 only. Small — make one existing input reactive (plus the fallback node only
if the spike requires it). Decoupled from #25; no interface change.

## Design history

Originally planned as identity-gated re-entry with transit-to-start, and briefly merged
into #25 (the preempt-vs-fail collision). After Roland relaxed transit-to-start to in-place
switch, the fix collapses to "make the path input track the current task" — no FAILURE, no
identity gate, decoupled from #25. Un-merged: this is standalone again; #25 is PR #37.
