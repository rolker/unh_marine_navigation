# Plan: Hardened task dispatch — contain matched-but-failed + identity-gated line re-entry

**Closes #25 and #35** (merged: both rewrite the same survey-line dispatch path; see
"Why merged"). Supersedes the standalone plans/PR #36.

## Issues

- https://github.com/rolker/unh_marine_navigation/issues/25 — SkipUnknownTaskType catchall silently marks tracklines done on subtree FAILURE
- https://github.com/rolker/unh_marine_navigation/issues/35 — Mission re-send mid-line doesn't take effect — BT latches FollowPath path on same-type task switch

## Why merged

#35's fix makes `SurveyLineTask` return FAILURE on a mid-line task switch (identity
gate). #25 adds `RetryUntilSuccessful` + `SetTaskFailed` around the same `survey_line`
case. The two intents flow through the *same* FAILURE signal — preemption ("re-enter
the new line") vs execution failure ("retry, then flag + skip"). Resolving that
collision *is* the work, and it lives in one stretch of dispatch XML + `SurveyLineTask`.
Doing them separately would stack a footgun for #35 to defuse. Merged per Q4.

## Context

`NavigatorSequence` (`run_tasks.xml:127-170`) dispatches via a `ReactiveFallback` whose
children are per-type subtrees gated by a first-child
`ScriptCondition(current_task_type == '<type>')`, with a trailing
`SetTaskDone "SkipUnknownTaskType"` catchall. Unmatched-type and matched-but-failed
share that one exit, so a `FollowPath` ABORT inside a matched `survey_line` falls
through to the catchall and the trackline is recorded clean-done (#25). Separately,
`SurveyLineTask` latches `{survey_path}` via `SetPathFromTask` once and re-enters only
on a `current_task_type` change — so a `survey_line → survey_line` switch updates the
heartbeat task but never the followed path (#35).

Findings that shape the approach (verified in this worktree):
- **Record surface already exists.** `TaskInformation.msg` has `string status`;
  `Task::setStatus`/`status()` exist in C++ (`task.cpp:98`) and Python (`task.py:128`);
  `bt_types.cpp:245` already serializes `status` onto the feedback/heartbeat JSON that
  **camp** renders. `setStatus` is never called today. So attempted-but-failed needs
  **no `.msg` change and no downstream rebuild** — just write the existing field (Q1).
- **No diagnostics infra exists.** Observability rides the existing status→heartbeat→camp
  path + `RCLCPP_ERROR`, not a new `DiagnosticStatus` publisher (Q1). Camp gaining a
  richer mission-status display is a separate camp-repo follow-up, not this PR.
- **Retry pattern is in-repo.** `RunSurveyAreaSubTasks` (`:198`) and `SurveyLineSetTask`
  (`:330`) already wrap line execution in `RetryUntilSuccessful num_attempts="3"`; both
  currently mark the parent done on exhausted retry (the same #25 bug, one level down).
- BT is `BTCPP_format="4"` (#21) → stock `Switch` available.

Decisions locked with Roland: retry then skip + flag, not halt (out-of-comms autonomy);
record attempted-but-failed distinctly from done; structurally distinct exits;
mirror at all three dispatch levels (Q3); merge with #35 (Q4).

## Approach

Single PR on `feature/issue-25`.

### Part A — contain matched-but-failed (#25)

1. **`Switch` dispatch.** Replace `NavigatorSequence`'s `ReactiveFallback` + per-subtree
   `ScriptCondition` gates with a `Switch` on `current_task_type`; the **default branch**
   is the existing `SetTaskDone "SkipUnknownTaskType"`. A matched case that FAILS no
   longer reaches the default — the core structural fix.
2. **New `SetTaskFailed` node** (sibling of `SetTaskDone`): writes a **structured**
   `status` (`{outcome: failed, reason: <follow_path_error_code>, attempts: N}` — a
   stable contract for camp), calls `setDone()` so the mission advances, and
   `RCLCPP_ERROR`s. Registered in `bt_register_nodes.cpp`.
3. **Per-case retry + flag at all three levels (Q3).** Wrap each matched case in
   `RetryUntilSuccessful num_attempts="3"` with a `SetTaskFailed` fallback, at:
   `NavigatorSequence`, `RunSurveyAreaSubTasks`, and `SurveyLineSetTask`. Effect at the
   nested levels: an aborted sub-line is recorded individually and the area/set
   **continues its remaining lines** (skip-and-continue) instead of abandoning the parent.

### Part B — identity-gated line re-entry (#35)

4. **`SetPathFromTask`** — add `OutputPort<std::string>("path_task_id")` emitting
   `task->message().id`, latched alongside `{survey_path}`. C++ latch, immune to
   blackboard-remap quirks.
5. **`SurveyLineTask` identity gate** — re-entry detector
   `ScriptCondition(current_task_id == path_task_id)` as a sibling of `FollowPath` inside
   a `ReactiveSequence`, re-evaluated each tick. On a mid-line switch it fails → halts
   `FollowPath` → re-latch → restart on the new line's path. First-entry guard for empty
   `path_task_id`. Log an INFO on halt/restart (transparency).
6. **Survey-area / line-set reuse** — `SurveyLineTask` is also instantiated under
   `RunSurveyAreaSubTasks` (id = `current_survey_area_task_id`) and `SurveyLineSetTask`
   (id = `current_survey_line_set_sub_task_id`). Wire the identity comparison to resolve
   to the correct id at each call site, or scope the gate so those paths stay correct.

### Part C — the preempt-vs-fail seam (Q4, the hard core)

7. **Preemption must not consume the retry budget or trip `SetTaskFailed`.** Candidate
   structure for the `survey_line` case:

   ```
   Fallback                                  # SetTaskFailed only on genuine exhaustion
     RetryUntilSuccessful num_attempts=3
       Sequence (memory)
         SetPathFromTask( survey_path, path_task_id := current_task_id )   # re-latches each (re)entry
         ReactiveSequence
           ScriptCondition( current_task_id == path_task_id )              # preemption detector
           FollowPath( survey_path )
         SetTaskDone
     SetTaskFailed
   ```

   A switch fails the inner `ScriptCondition` → `FollowPath` halted → memory `Sequence`
   FAILS → `RetryUntilSuccessful` re-ticks → `SetPathFromTask` re-latches the new line →
   `FollowPath` restarts on the new path. **Known subtlety (validate, don't hand-wave):**
   this consumes one of the 3 attempts per switch, so ≥3 rapid switches could trip
   `SetTaskFailed` on a non-failure. If the re-entry fixture (below) shows that matters,
   separate the preemption path from the retry counter (e.g. reset on id-change, or route
   preemption above the retry). This is the PR's central design risk and what the
   re-entry test exists to pin down.

### Part D — tests (Q2; minimal inline now, no #8 dependency)

8. **`SetTaskFailed` node gtest** — asserts it sets `done` **and** writes a non-empty
   structured `status`; contrast `SetTaskDone` leaving status empty. Adds
   `ament_cmake_gtest` infra to `marine_nav_behavior_tree`.
9. **Routing fixture** — minimal BT-XML: a matched case forced to FAILURE routes to
   `SetTaskFailed` (status set, not clean-done); an unmatched type hits the default
   catchall. Self-contained in BT.CPP with stub task nodes.
10. **Re-entry fixture** — tick `survey_line` with a mid-line `current_task_id` change;
    assert `{survey_path}`/`path_task_id` re-latch to the new line and the line is **not**
    recorded `failed` (the Part C seam). Drives resolution of the retry-budget subtlety.

Full-tree integration (loading the real `run_tasks.xml` under a running navigator) is
deferred to #8's launch_testing harness.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | `Switch` dispatch; retry + `SetTaskFailed` at 3 levels; `SurveyLineTask` identity gate + re-latch structure (Part C); INFO log; embedded `TreeNodesModel` for `SetTaskFailed` |
| `marine_nav_behavior_tree/.../action/set_task_failed.{h,cpp}` | **New** node: structured `setStatus` + `setDone` + `RCLCPP_ERROR` |
| `marine_nav_behavior_tree/.../action/set_path_from_task.{h,cpp}` | Add `path_task_id` output port; emit `task->message().id` |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register `SetTaskFailed` |
| `marine_nav_behavior_tree/CMakeLists.txt` + new `test/` | Add node to plugin lib; gtest infra; node + routing + re-entry tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control & transparency | Failures recorded (structured status) + logged + on camp heartbeat; preemption logged. No silent mark-done. |
| A change includes its consequences | Reuses existing `status` (no interface/downstream ripple); all three dispatch levels fixed; survey-area/line-set reuse covered; tests land with the code. |
| Test what breaks | The break is dispatch routing + path re-latch — both get focused BT fixtures, not just node tests. |
| Only what's needed | No new `.msg`, no diagnostics publisher; stock `Switch`/`Retry` + one node + one port. |
| Capture decisions | Q1–Q4 rationale here + `progress.md` + PR body (no `docs/decisions/` in repo). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | `feature/issue-25` worktree; PR into `jazzy`; no direct default-branch commits. |
| 0008 — ROS 2 conventions | Yes | Stock BT.CPP4 `Switch`/`RetryUntilSuccessful`; new node/port follow existing `SetTaskDone`/`SetPathFromTask` idioms; reuses the existing `status` channel. |

## Consequences

| If we change... | Also update... | In this PR? |
|---|---|---|
| Write `TaskInformation.status` for the first time | Cross-repo grep (`mission_manager`, `rqt_*`, camp) for any consumer assuming `status` stays empty | Audit before merge; no `.msg` change |
| `Switch` dispatch + retry at 3 levels | `TreeNodesModel` / generated `*_nodes.xml`; verify each level's `SetTaskDone` still fires on success | Yes |
| `SetPathFromTask` ports | Every `run_tasks.xml` call site of `SetPathFromTask` | Yes — audit call sites |
| Mid-line preemption semantics | README BT-flow docs if present | Yes if present |

## Open Questions

All four planning questions resolved with Roland (Q1 status-field+log; Q2 routing
fixture now; Q3 all three levels; Q4 merge with #35). Remaining items are
**implementation/validation**, not design forks:
- [ ] Retry-budget vs preemption (Part C step 7) — confirm via the re-entry fixture whether
  a switch consuming a retry attempt is acceptable, or separate the counter.
- [ ] `current_task_id` resolution at the survey-area / line-set call sites (step 6) — verify under each remapping.

## Estimated Scope

Single PR in `unh_marine_navigation`, closing #25 + #35. Moderate: one new node + one
new port, a `Switch`-based dispatch rewrite, retry/flag at three levels, the identity
re-entry structure, three BT test fixtures. **No interface change, no downstream
rebuild.** Central risk is the Part C preempt-vs-fail seam, pinned by the re-entry test.
