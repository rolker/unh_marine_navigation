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

## Open Questions

- [ ] **Bundle Phase 2 here or stack as a follow-up?** Recommend **stacking** — Phase 1 fully
      fixes the reported end-of-task symptom and is small/safe; Phase 2 is a distinct
      reactive-refresh design (re-entry-on-id-change + reactive action node) with its own test
      matrix, mirroring how #35 was its own change. (Counter: the "bundle related cleanups"
      preference — your call.)
- [ ] **Phase 2 mechanism:** acceptable to add a small reactive id-latch in the BT XML, or do
      you prefer a dedicated custom condition node (`TaskIdChangedCondition`) for clarity/testing?
- [ ] Should `done_hover` instead carry the last survey pose explicitly, sidestepping the
      drift-projection path entirely? (Leaning no — drift-to-stop is the intended #33 behavior.)

## Estimated Scope

Phase 1: single small PR (XML one-line swap + comment + one gtest). Phase 2: a second stacked
PR if approved. Defaulting to Phase 1 this PR pending the bundle-vs-stack answer.
