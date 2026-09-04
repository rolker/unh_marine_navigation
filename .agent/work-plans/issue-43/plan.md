# Plan: Different-task mid-mission re-send must transit to the new task's start

**Closes #43.** Follow-up to #35 / PR #36 (which landed the reactive `SetPathFromTask` and inherited a plan-wording bug that over-broadened the in-place-switch decision).

## Issue

https://github.com/rolker/unh_marine_navigation/issues/43

The reactive `SetPathFromTask` introduced in PR #36 refreshes `{survey_path}` from `{survey_line_task}` every tick. When the operator re-sends a mission with a **different task** (different `task_id`), the new task immediately replaces the old via `UpdateCurrentTaskData` upstream, the reactive refresh propagates the new path, and Nav2 `FollowPath` preempts onto the new path **from the boat's current location** — no transit-to-start.

**Roland's actual design decision** (recorded 2026-05-26, mis-captured in #35's plan):
- **Same task with content modification** → continue in-place (PR #36 behavior, applied selectively to this case).
- **Different task** → MUST transit to its start (run `TransitAndSurveyLine` for the new task, not just refresh the path).

The over-broad "in-place switch" wording in the #35 plan collapsed both cases into one. PR #36's implementation inherited the over-broad reading.

## Context — the autoremap problem

`SurveyLineTask` (`run_tasks.xml:424-437` post-#37):

```xml
<BehaviorTree ID="SurveyLineTask">
  <ReactiveSequence>
    <ScriptCondition code="current_task_type == 'survey_line'"/>
    <Sequence>
      <SetPathFromTask task="{survey_line_task}" .../>          <!-- one-shot at sequence entry; #36 added a reactive sibling inside SurveyLine -->
      <SubTree ID="TransitAndSurveyLine" _autoremap="true"/>     <!-- transit (once on entry) + SurveyLine (ReactiveSequence) -->
      <SetTaskDone task="{survey_line_task}"/>                   <!-- BUG: marks whatever {survey_line_task} points to NOW (potentially the new task) -->
    </Sequence>
  </ReactiveSequence>
</BehaviorTree>
```

The autoremap chain `{survey_line_task}` aliases `{current_task}` at the top-level caller (`run_tasks.xml:159`). When `UpdateCurrentTaskData` advances `{current_task}` to a different task pointer (because the operator re-sent a mission), `{survey_line_task}` *immediately* reflects the new pointer. Any operation in `SurveyLineTask` that uses `{survey_line_task}` reads the NEW task, not the one originally entered:

- **`SetTaskDone task="{survey_line_task}"` at the end of the Sequence**: marks the NEW task done (wrong — the new task hasn't run yet).
- **`SetTaskFailed task="{current_task}"` at the dispatch level** (`run_tasks.xml:183`, post-#37): if `SurveyLineTask` returns FAILURE for any reason, marks the NEW task failed (wrong — the new task wasn't the one that failed).

So **even with PR #36's reactive refresh**, the bookkeeping is broken on a task switch — and any clean fix that requires "exit `SurveyLineTask` and re-enter for the new task" must first solve the bookkeeping problem.

## Approach

Two-part fix, sized as a single PR on `feature/issue-43`:

1. **Capture the entry-time task pointer at `SurveyLineTask` entry; act on the captured pointer for terminal operations.** New C++ BT node `CaptureTask` that latches a `TaskPtr` from a port into a blackboard slot on first tick, and is a no-op on subsequent ticks. `SetTaskDone` / `SetTaskFailed` at the `SurveyLineTask` level read the captured pointer instead of the live `{survey_line_task}`. This decouples the bookkeeping from the autoremap chain — switching `{current_task}` mid-execution no longer corrupts which task is marked done/failed.

2. **Detect task switch + clean re-entry to `TransitAndSurveyLine`.** New C++ BT condition node `TaskIdUnchanged` (or reuse `CaptureTask` outputs + a `Script` comparison if BT.CPP scripting can compare task IDs — see Open Q1). When the current `{survey_line_task}->id()` differs from the captured `entry_survey_line_task->id()`, return FAILURE from a wrapper inside `SurveyLine`'s `ReactiveSequence`, causing the inner `Sequence` to fail. **The wrapper differentiates "switch failure" from "real failure" via a blackboard flag (`switch_pending`)**, so the outer dispatch routes to clean re-entry rather than `SetTaskFailed`.

### Detailed BT XML restructuring

Proposed post-fix `SurveyLineTask` shape (changes from current marked with `<!-- NEW -->`):

```xml
<BehaviorTree ID="SurveyLineTask">
  <ReactiveSequence>
    <ScriptCondition code="current_task_type == 'survey_line'"/>
    <Fallback name="SurveyLineTaskOrSwitch">                                <!-- NEW: switch-vs-real-fail differentiator -->
      <Sequence>
        <CaptureTask task="{survey_line_task}"                              <!-- NEW: latch entry task pointer -->
                     captured_task="{entry_survey_line_task}"/>
        <Script code="switch_pending := false"/>                            <!-- NEW: reset flag -->
        <SetPathFromTask task="{entry_survey_line_task}" .../>              <!-- one-shot, on captured task -->
        <SubTree ID="TransitAndSurveyLine"
                 entry_survey_line_task="{entry_survey_line_task}"          <!-- NEW: explicit forwarding -->
                 _autoremap="true"/>
        <SetTaskDone task="{entry_survey_line_task}"/>                      <!-- NEW: act on captured pointer -->
      </Sequence>
      <Sequence name="SwitchReenter">                                       <!-- NEW: only runs if inner Sequence failed -->
        <ScriptCondition code="switch_pending == true"/>                    <!-- only proceed if failure was a switch -->
        <!-- Returning SUCCESS here means SurveyLineTask returns SUCCESS without
             marking captured task done. Dispatch picks up current_task (the
             switched-to task) on next tick → re-enters SurveyLineTask fresh
             with the new task pointer → CaptureTask latches the new entry. -->
        <AlwaysSuccess/>
      </Sequence>
    </Fallback>
  </ReactiveSequence>
</BehaviorTree>
```

Inside `SurveyLine`'s `ReactiveSequence` (where the reactive `SetPathFromTask` from #36 lives), add a switch detector:

```xml
<BehaviorTree ID="SurveyLine">
  <Sequence>
    <SubTree ID="CancelAllNavigation" _autoremap="true"/>
    <ReactiveSequence>
      <Script code="navigation_state := 'survey_line'"/>
      <ControllerSelector .../>
      <SetControllerSpeed .../>
      <Fallback name="SwitchDetectOrContinue">                              <!-- NEW -->
        <Sequence>
          <TaskIdUnchanged task="{survey_line_task}"                        <!-- NEW: SUCCESS if unchanged, FAILURE if changed -->
                           reference="{entry_survey_line_task}"/>
          <SetPathFromTask task="{survey_line_task}" .../>                  <!-- PR #36 reactive refresh — only runs when id unchanged -->
        </Sequence>
        <Sequence name="SwitchSignal">                                      <!-- runs when TaskIdUnchanged failed -->
          <Script code="switch_pending := true"/>                           <!-- NEW: flag the outer Fallback -->
          <!-- Failure here halts the ReactiveSequence early; FollowPath
               will be canceled by the BT halt + the SurveyLineTask outer
               Fallback re-routes via the SwitchReenter branch. -->
          <AlwaysFailure/>
        </Sequence>
      </Fallback>
      <RecoveryNode .../>                                                   <!-- existing #37 wrap unchanged -->
    </ReactiveSequence>
  </Sequence>
</BehaviorTree>
```

This sequencing gives Roland's exact intent:

- **Same task, modified content**: `TaskIdUnchanged` returns SUCCESS; the reactive `SetPathFromTask` refreshes `{survey_path}` from `{survey_line_task}`; FollowPath sees the new path via `on_wait_for_result`, preempts in-place. PR #36's behavior, applied **only** to this case.
- **Different task**: `TaskIdUnchanged` returns FAILURE; the `Fallback` runs the switch branch which sets `switch_pending=true` and emits FAILURE; `SurveyLine`'s `ReactiveSequence` returns FAILURE; `TransitAndSurveyLine` returns FAILURE; the inner `Sequence` in `SurveyLineTask` returns FAILURE; the outer `Fallback` in `SurveyLineTask` runs the `SwitchReenter` branch which checks `switch_pending`, then returns SUCCESS (without marking the captured task done). On the next dispatch tick, `current_task` is the new task → routes back into `SurveyLineTask` → `CaptureTask` latches the new entry → `TransitAndSurveyLine` runs transit + survey for the new task.

### Three call sites for SurveyLineTask

All three pass a different key into `survey_line_task`:

- **Top level** (`:159`): `survey_line_task="{current_task}"` → captured as `{entry_survey_line_task}`.
- **`SurveyAreaTask`** loop (`:248`): `survey_line_task="{current_survey_area_task}"` → captured as `{entry_survey_line_task}`. The outer loop's `UpdateCurrentTaskData` re-selects each tick, so the same switch-detection logic applies one level down.
- **`SurveyLineSetTask`** loop (`:406`): `survey_line_task="{survey_line_set_sub_task}"` → captured as `{entry_survey_line_task}`. Same shape.

The `entry_survey_line_task` blackboard slot is per-`SurveyLineTask`-instance (BT.CPP subtree blackboards isolate by call), so no cross-site interference.

### New C++ BT nodes

**`CaptureTask`** (`marine_nav_behavior_tree/plugins/action/capture_task.{h,cpp}`):

- `SyncActionNode`
- Inputs: `task` (`TaskPtr`)
- Outputs: `captured_task` (`TaskPtr`)
- Behavior: on first tick (when `captured_task` is null/uninitialized), copy `task` to `captured_task` and return SUCCESS. On subsequent ticks, leave `captured_task` unchanged and return SUCCESS. Reset on `halt()`.
- Test: latches first task, ignores subsequent task changes until halt; halt clears the captured slot.

**`TaskIdUnchanged`** (`marine_nav_behavior_tree/plugins/condition/task_id_unchanged.{h,cpp}`):

- `ConditionNode`
- Inputs: `task` (`TaskPtr`), `reference` (`TaskPtr` — the captured pointer)
- Behavior: returns SUCCESS if `task->id() == reference->id()`, FAILURE otherwise. Handles null reference (returns SUCCESS as a safe default — captured slot not yet populated).
- Test: pre-capture (null reference) → SUCCESS; same-id → SUCCESS; different-id → FAILURE.

Both nodes registered in `marine_nav_behavior_tree/src/bt_register_nodes.cpp` and added to `marine_nav_behavior_tree/CMakeLists.txt`.

### Tests

- `marine_nav_behavior_tree/test/test_capture_task.cpp` — unit test for `CaptureTask` (latch on first tick; ignore subsequent; halt resets).
- `marine_nav_behavior_tree/test/test_task_id_unchanged.cpp` — unit test for `TaskIdUnchanged`.
- `marine_nav_behavior_tree/test/test_survey_line_task_switch.cpp` — routing fixture (mirrors `test_dispatch_routing` pattern from #25/PR#37). Mock `FollowPath` action server (or stub via `BT::SyncActionNode`), drive `SurveyLineTask` through:
  1. Enter with task A → expect `entry_survey_line_task` captured + FollowPath dispatched with task A's path.
  2. Mid-tick swap `{survey_line_task}` to task B (different id) → expect `switch_pending=true`, `SurveyLineTask` returns SUCCESS (without `SetTaskDone(A)` running).
  3. Next tick → captured slot reset (via halt) → re-enter with task B.

### Sim verification

Re-run the PR #36 sim setup (`marine_simulation simulator_launch.py`, camp drive). Upload a 2-line mission; mid-line on line 1, re-send with a different second line at a meaningful distance/bearing. Confirm via bag of `/ben/follow_path/_action/status`, `/ben/run_tasks/_action/feedback`, `/ben/marine/heartbeat`:

- The OLD task's `SetTaskDone` does NOT fire (heartbeat does not flip the old task's status to "done").
- A NEW transit leg (`NavigateThroughWaypoints` → goal_pose change → `compute_path_through_poses` invocation) runs for the new task.
- The boat traverses to the new line's start, then surveys it normally.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_behavior_tree/include/marine_nav_behavior_tree/plugins/action/capture_task.h` | New: `CaptureTask` SyncActionNode declaration. |
| `marine_nav_behavior_tree/src/plugins/action/capture_task.cpp` | New: latch-on-first-tick implementation + `halt()` reset. |
| `marine_nav_behavior_tree/include/marine_nav_behavior_tree/plugins/condition/task_id_unchanged.h` | New: `TaskIdUnchanged` ConditionNode declaration. |
| `marine_nav_behavior_tree/src/plugins/condition/task_id_unchanged.cpp` | New: id-comparison implementation; safe on null reference. |
| `marine_nav_behavior_tree/src/bt_register_nodes.cpp` | Register both new nodes. |
| `marine_nav_behavior_tree/test/test_capture_task.cpp` | New: unit test (3+ cases). |
| `marine_nav_behavior_tree/test/test_task_id_unchanged.cpp` | New: unit test (3+ cases). |
| `marine_nav_behavior_tree/test/test_survey_line_task_switch.cpp` | New: routing fixture (3+ cases — capture, switch, re-enter). |
| `marine_nav_behavior_tree/CMakeLists.txt` | Add the two new source files + three new GTest blocks. |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Restructure `SurveyLineTask` (Fallback + Sequence + SwitchReenter); add switch-detect Fallback inside `SurveyLine`'s `ReactiveSequence`; update `TreeNodesModel` for `CaptureTask` + `TaskIdUnchanged`. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions | Roland's narrower decision (same-task-modified-in-place vs different-task-must-transit) is captured in the plan Context with the failure mechanism explained, not assumed. |
| A change includes its consequences | The autoremap bookkeeping bug is part of the same fix scope (SetTaskDone / SetTaskFailed on wrong task); not deferred. |
| Only what's needed | Two minimal BT nodes (latch + id-compare); reuses existing `SetTaskDone` / `SetPathFromTask` infra; no controller / planner changes. |
| Test what breaks | Unit tests for both new nodes; routing fixture exercises the capture + switch + re-enter cycle. Sim verification confirms operator-visible behavior. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Work in `layers/worktrees/issue-unh_marine_navigation-43/`. |
| 0008 — Follow ROS 2 conventions | Yes (lightly) | New BT nodes follow the existing `marine_nav_behavior_tree` pattern (header + cpp + bt_register_nodes registration + ament_add_gtest). |
| 0013 — `progress.md` vocabulary | Yes | Lifecycle: Plan Authored → Plan Review → Implementation → Local Review (Pre-Push) → Integrated Review. |

## Consequences

| If we change... | Also update... | Status |
|---|---|---|
| `SurveyLineTask` structure (`Fallback` + `SwitchReenter` branch) | All three callers of `SurveyLineTask` (top-level, `SurveyAreaTask`, `SurveyLineSetTask`) — verify the `SwitchReenter`'s SUCCESS-without-SetTaskDone behavior is correct for each loop's expectations | Verify in tests; the loops re-tick `UpdateCurrentTaskData` each iteration, so a switch + clean re-entry is consistent with their existing dispatch model. |
| Add `entry_survey_line_task` blackboard key | Document the new key in the plan's BT XML comments; ensure nested `SurveyLineTask` calls inside `SurveyAreaTask` / `SurveyLineSetTask` have their OWN per-instance blackboard slot (BT.CPP subtree isolation handles this automatically). | Verified by routing fixture: each subtree call has its own blackboard. |
| `bt_register_nodes.cpp` + `CMakeLists.txt` | TreeNodesModel block in `run_tasks.xml` (declare `CaptureTask` + `TaskIdUnchanged` as Action/Condition entries) | In plan. |
| Operator semantics | Heartbeat behavior: on a different-task switch, the OLD task should NOT show "done"; the NEW task should show transit-then-survey progression. Document this in PR body so reviewers know what camp will display. | In PR body. |

## Open Questions

- **Q1 (`TaskIdUnchanged` vs BT.CPP scripting)**: BT.CPP v4 scripting (`Script` / `ScriptCondition`) operates on JSON-serializable scalars. `TaskPtr` is a `std::shared_ptr<marine_nav_tasks::Task>` — not directly scriptable. The cleanest path is a new C++ condition node (`TaskIdUnchanged`); a script-based approach would require exposing `task->id()` as a separate `string` blackboard key, which is more glue code than just writing the comparator node.
- **Q2 (Halt semantics for `CaptureTask`)**: when `SurveyLineTask`'s outer `ReactiveSequence` halts (e.g. because the operator switches to a non-`survey_line` task type), should `CaptureTask::halt()` clear the captured slot, or leave it for next entry? Decision: **clear** — fresh entry should always re-latch. Alternative would risk stale capture leaking between unrelated `SurveyLineTask` instances.
- **Q3 (Test infrastructure)**: PR #37 added `test_dispatch_routing.cpp` as a routing fixture; the planned `test_survey_line_task_switch.cpp` mirrors its pattern. Reuse the fixture scaffolding; do NOT add new mock-action-server infrastructure for this PR (defer that to a future general-purpose harness if it ever justifies the scope).
- **Q4 (Coordination with #40 / PR #40)**: PR #40 changes `SurveyLine`'s entry from `CancelAllNavigation` → `ExitHoverForNav` (no `CancelFollowPath`). The switch-detect path here causes the inner `Sequence` to return FAILURE, which halts the `ReactiveSequence` and indirectly halts `FollowPath` via BT teardown. With PR #40 in place, the halt is faster (no explicit CancelFollowPath, just the BT halt + next FollowPath goal preempts). Confirm the sim sees no zero-cmd window introduced by the switch path specifically — this is one of #40's continuity-test follow-ups.

## Estimated Scope

Moderate. ~3-5 hours focused work:

- 2 new BT C++ nodes + tests (~200-300 LoC including tests): 1.5-2 hr.
- BT XML restructuring (Fallback + SwitchReenter + switch-detect inside SurveyLine + entry_survey_line_task forwarding): 0.5-1 hr (careful work, easy to get wrong).
- Routing-fixture test (~100-150 LoC): 0.5-1 hr.
- Sim verification (same harness as PR #36): 0.5-1 hr.
- PR body + progress.md + review-code pre-push: 0.5 hr.

Coordinate landing AFTER PR #36 (the reactive `SetPathFromTask` is what the switch-detect needs to override on same-task vs different-task). Land before deployment if June 4 freeze allows; otherwise the deployment runs with PR #36's known-limitation behavior (different-task switch = in-place pickup), documented in PR #36's body.
