# Plan: Hover sticks to first-evoked location — SequenceWithMemory never re-ticks PredictStoppingPose on re-entry

## Issue

https://github.com/rolker/unh_marine_navigation/issues/46

## Context

`HoverTask` (`run_tasks.xml:64-95`) wraps `[Fallback, PredictStoppingPose, Script, Hover]`
in `SequenceWithMemory`. In BT.CPP 4.9.0 that node resets its child index only on
`SUCCESS`; its `halt()` is understood **not** to reset the index (the documented contract
that distinguishes it from plain `Sequence`, whose `halt()` resets to 0). Note the
`halt()` bodies live in the compiled lib — not readable here — so this distinction is
asserted from BT.CPP's documented behavior and is **pinned by the Phase-1 regression test**
(step 3), not read from source. `Hover` is a perpetual station-keep that never returns
`SUCCESS` (`hover.cpp:205`), so the index never resets:
after the first engagement the node parks at the `Hover` child and every later re-entry
(via `Switch5` halting/re-selecting on `current_task_type`) resumes straight at `Hover`,
**skipping `PredictStoppingPose`**. `{hover_target}` is therefore computed once — the
first time hover is evoked (at activation that's the launch point, since `done_hover` is
the only task) — and reused forever. `PredictStoppingPose` and the hover server are both
correct in isolation; the defect is the control-node choice. The XML comment at
`run_tasks.xml:79-84` documents the opposite of the actual behavior.

## Approach

**Phase 1 (this PR) — end-of-task / `done_hover` fix + regression test**

1. **Swap `SequenceWithMemory` → `Sequence` in `HoverTask`** (`run_tasks.xml`). Plain
   `Sequence::halt()` resets the index to 0, so each re-entry re-runs `Fallback` +
   `PredictStoppingPose` (fresh stop point), while a *continuous* hold still resumes at the
   RUNNING `Hover` child (no per-tick recompute — important: recomputing every tick would
   chase the drifting boat, which is why the stop point must be captured once *per entry*).
   The continuous-hold safety has a load-bearing dependency: it holds because the inner
   sequence is the **last** child of the outer `ReactiveSequence` (`run_tasks.xml:62`), so
   ReactiveSequence (which halts only siblings *after* a RUNNING child) never halts it
   between loops. A future edit that adds a sibling after the sequence would break this —
   call this out in the comment (step 2).
2. **Rewrite the `run_tasks.xml:79-84` comment** to state the real contract: stop point is
   recomputed once per entry because plain `Sequence` resets on halt; continuous hold is
   safe only while the sequence stays the last child of the ReactiveSequence; do **not**
   restore `SequenceWithMemory` (links the failure mode to this issue).
3. **Add a regression gtest** that pins **both** halves of the contract this fix depends on:
   minimal tree `Sequence[counting-sync-action, never-succeeding(RUNNING)-leaf]`, then
   (a) tick → `haltTree` → tick, asserting the counting action ran **twice** (re-entry
   recompute — the bug); and (b) tick → tick with **no** halt, asserting it ran **once**
   (continuous-hold no-recompute — the regression guard). The same (a) built with
   `SequenceWithMemory` asserts **once** (documents why the swap is load-bearing). No ROS
   fixture needed — pure BT-mechanics test. Place it in `marine_nav_behavior_tree/test/`
   (which already has a gtest harness — 6 tests), **not** `bt_task_navigator` (which has no
   test infrastructure at all), to avoid bootstrapping a new test target.

**Phase 2 — same-type re-command (`hover_override`) — see Open Questions for bundle-vs-stack**

4. The pure same-type re-command (operator re-hovers while already hovering; `current_task_type`
   stays `hover`, so `Switch5` never halts) is **not** fixed by Phase 1 and **not** fixed by
   re-reading the target port alone: with no halt the `Sequence` stays parked at `Hover`, so
   `Fallback`/`PredictStoppingPose` never re-tick and `{hover_target}`/`{goal_poses}` never
   refresh. The real lever is **forcing HoverTask re-entry when `current_task_id` changes**
   (a reactive id-latch guard in the BT), paired with making `HoverAction` reactive
   (`on_wait_for_result` re-reads `target`, sets `goal_updated_` → `send_new_goal()`, the
   #35/FollowPath pattern) so a re-entry recomputed target is actually re-sent mid-RUNNING.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | P1: `SequenceWithMemory`→`Sequence` in `HoverTask`; correct comment. P2: reactive id-latch guard. |
| `marine_nav_behavior_tree/test/test_sequence_reentry.cpp` (new) | P1: re-entry-recompute + continuous-hold-no-recompute regression test (Sequence vs SequenceWithMemory). Lives here because this package already has a gtest harness. |
| `marine_nav_behavior_tree/CMakeLists.txt` | P1: add the new gtest to the existing test block. |
| `marine_nav_behavior_tree/src/plugins/action/hover_action.{cpp,h}` | P2 only: override `on_wait_for_result` for reactive target re-send. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Quality Standard — fix completely, add the test | Phase 1 ships the fix + a regression test that fails on the old node and passes on the new one. |
| Never circumvent tests; fix code not config | Behavior fix in the tree, not a test/config workaround. |
| Documentation accuracy | The misleading `run_tasks.xml:79-84` comment is corrected, not left to drift. |
| Surface scope deferrals | Phase 2 split decision is raised as an Open Question, not silently deferred. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 (progress.md vocabulary) | Yes | Plan/progress entries follow the schema (handled by plan-task skill). |
| Functional ADRs | No | A BT control-node swap + reactive action node triggers none. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `run_tasks.xml` (installed via ament `install(DIRECTORY)`, `CMakeLists.txt:51`) | Rebuild `marine_nav_bt_task_navigator` — symlink-install does **not** pick up XML data-file edits | Yes — note in test/build steps |
| `HoverTask` control flow | **Cross-repo** (`unh_marine_autonomy`, not this repo): `done_hover` is emitted by `mission_manager.py` and exercised by `marine_autonomy_integration_tests/test/test_mission_navigation_failure.py`. The swap doesn't change task semantics, only when `PredictStoppingPose` re-ticks, so no `unh_marine_autonomy` change is expected — but flag a manual cross-repo integration check before merge. | Manual cross-repo check (not in this PR's CI) |
| Hover re-entry semantics | Existing `test_predict_stopping_pose.cpp` (unaffected — node unchanged) | Verified no change |

## Open Questions (resolved 2026-05-29)

- [x] **Bundle Phase 2 here or stack?** → **Bundle** (operator wants one fix for a same-day
      sim test + deployment).
- [x] **Phase 2 mechanism?** → **Custom C++ node** (`RestartOnTaskChange` decorator), not inline
      XML — for unit-testability and to avoid the XML-scripting footgun of routing a failed
      condition to `SetTaskFailed`.
- [x] **`done_hover` explicit pose?** → **No.** Keep drift-to-stop (intended #33 behavior).

## Implementation Notes

- Phase 2 restart key is **`{current_task_update_time}`**, not task id: a re-issued
  `hover_override` keeps the same id ('hover_override') and only its pose changes, so an
  id-only check would miss it. `Task::update` bumps `last_update_time_` only when the task
  message actually changes (`task.cpp:44`), so a benign feedback-loop re-send of an identical
  task does not trigger a spurious restart (which would thrash hover). The quick sim test is
  the gate that confirms no thrash in practice.
- The `RestartOnTaskChange` decorator's `haltChild()` + re-tick cancels and re-sends the hover
  goal on a re-command, so the separately-considered `HoverAction::on_wait_for_result` reactive
  change was **not needed** — fewer files touched.
- Regression test placed in `marine_nav_behavior_tree/test/test_hover_reentry.cpp` (existing
  gtest harness). 5 cases, all green: plain-Sequence re-ticks after halt; SequenceWithMemory
  does not (documents the bug); plain-Sequence no-recompute during continuous hold; decorator
  restarts on timestamp change; decorator stable when timestamp unchanged.
- **Local lint note:** `colcon test` `ament_uncrustify` fails package-wide, including committed
  files this PR does not touch — a local uncrustify version skew, not introduced here. Not
  mass-reformatting per the no-circumvent-tests rule; gtests + build are green.

## Estimated Scope

Single PR, both phases. Implemented and unit-tested; pending quick sim test before merge/deploy.
