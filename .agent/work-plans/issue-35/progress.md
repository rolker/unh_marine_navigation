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

## Plan Review (post-redesign, in-place switch)
**Status**: complete
**When**: 2026-05-27 12:04 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — independent fresh-context sub-agent review

**Plan**: `.agent/work-plans/issue-35/plan.md` at `0a681d6`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/36
**Verdict**: changes-requested (ready-with-fixes — plan refinements, not approach rework)

Verified correct against source: root cause (latch + type-only re-entry); `SetPathFromTask`
pure SyncAction (`set_path_from_task.cpp:73`); `UpdateCurrentTask` re-selects each tick;
transit-leg-not-re-run; clean #25 decoupling (no FAILURE produced); the Nav2 resend
assumption is correctly located and honestly flagged (BtActionNode reads ports only at
goal-send; RUNNING resend depends solely on `FollowPathAction::on_wait_for_result`, whose
`.cpp` isn't shipped — spike+fallback is the right de-risk).

### Findings
- [ ] (must-fix) Name the actual wiring, don't defer it: the reactive `SetPathFromTask` inside `SurveyLine` (`run_tasks.xml:282-302`) reads the task via the key `survey_line_task` carried down through the `_autoremap` chain (`TransitAndSurveyLine`→`SurveyLine`), NOT `{current_task}` directly. State the key + the autoremap dependency in the plan — leaving placement to "confirm during implementation" is the same class of gap that sank the prior version one layer up.
- [ ] (must-fix) Schedule the regression test or explicitly mark it deferred: no BT-task-navigator harness exists (only `marine_nav_behavior_tree/test/test_set_controller_speed_resolve.cpp`). The plan references a re-entry fixture in the Principles table but no Approach/Files step creates it, and the "ride #8's harness" framing was dropped without replacement. Commit to a minimal fixture (mock follow_path server, switch task mid-RUNNING, assert `{survey_path}` + dispatched goal track the new task) or state the deferral + why.
- [ ] (suggestion) Step-3 prose says "autoremapped `{current_task}`" but the real port is `survey_line_task` (top: `{current_task}`@150; area: `{current_survey_area_task}`@200; set: `{survey_line_set_sub_task}`@332) — name the real key to avoid wiring `{current_task}`.
- [ ] (suggestion) Step-4 "INFO log on path re-send" has no log node in this tree (no `LogText`); resolve to "Nav2 already logs the goal resend" (verify) or accept a tiny node and adjust the "no new node" framing.
- [ ] (suggestion) Spike should record the goal-UUID/BT-log change to positively confirm a resend (vs silently following a stale goal), not infer from boat motion.
- [ ] (suggestion) In-place-switch edge cases: a re-sent line far away / opposite-direction means FollowPath joins at an arbitrary nearest point with no lead-in (possible sharp turn / wrong traversal direction). Add one sentence to Consequences before field use.

## Implementation
**Status**: complete
**When**: 2026-05-28 09:35 -04:00 (code) → 2026-05-28 09:50 -04:00 (sim verification)
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Branch**: `feature/issue-35`
**Diff scope**: single-file BT XML edit — `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml`.

### Spike answered (Approach step 1)
Source-read `nav2_behavior_tree/plugins/action/follow_path_action.cpp` on the upstream `jazzy` branch (not shipped in `/opt/ros/jazzy`, but reachable via `gh api /repos/ros-navigation/navigation2/contents/...`). `FollowPathAction::on_wait_for_result` reads the `path` port every RUNNING tick and `if (goal_.path != new_path && new_path != Path()) { goal_.path = new_path; goal_updated_ = true; }`. `BtActionNode` then calls `send_new_goal()` on the next tick, preempting via the action server. **Auto-resend confirmed** — no explicit detect-change → cancel → resend fallback is required. The plan's Step 1 "Do not build past this without the answer" gate is satisfied.

### Wiring
Inserted a reactive `SetPathFromTask task="{survey_line_task}" path="{survey_path}" start_index="0" end_index="-1"` as a child of `SurveyLine`'s `ReactiveSequence` (`run_tasks.xml:295-309`), positioned just before `FollowPath`. The `ReactiveSequence` re-ticks every iteration, so `{survey_path}` is refreshed on each tick from whatever task `{survey_line_task}` currently points to — exactly what the plan called for. Autoremap chain holds: `SurveyLineTask:357-368` defines `survey_line_task` → `TransitAndSurveyLine:381-385` invokes `SurveyLine` with `_autoremap="true"` → `survey_line_task` propagates into `SurveyLine`'s blackboard.

### Sim verification (Approach step 2 — operator-driven)
Ran `marine_simulation simulator_launch.py` on the worktree install; Roland drove camp / rqt to upload + re-send missions; I instrumented with `ros2 bag record --include-hidden-topics` on `/ben/follow_path/_action/status`, `/ben/run_tasks/_action/status`, `/ben/cmd_vel_nav`, `/ben/marine/heartbeat`, etc. Bag at `.scratchpad/issue-35-sim/midline_switch_0.mcap` (1.7 MiB, 278 s, 09:45:43 – 09:50:21).

**What the bag confirms:**
- Operator mission re-send produced a clean `CANCELING → CANCELED → new EXECUTING` preempt cycle on `/ben/follow_path/_action/status` (window at 09:46:38 — old goal `71f69d71` → CANCELED, new `3132a54b` / `22e33b0a` EXECUTING). **No `STATUS_ABORTED` from controller failure** during the recorded window.
- Natural multi-line transitions within a single `run_tasks` goal completed cleanly (`a801ad80` SUCCEEDED → `a8d6cbd7` EXECUTING at 09:48:29 — line 1 done, BT advanced to line 2). The reactive `SetPathFromTask` did not break this flow.
- The two `STATUS_ABORTED` events I see on `/ben/follow_path/_action/status` (at 09:48:40 and 09:48:55) are immediately preceded by a `run_tasks` ABORTED (operator restarted the mission). These are BT-halt propagations through `CancelFollowPath`, not controller failures — the new `follow_path` goal was already `EXECUTING` alongside.
- Direct operator confirmation (Roland): "when I modified the line and resent, it worked fine continuing on the line" — that is the exact scenario the fix is for, and it produced the expected continue-on-the-modified-line behavior, not the pre-fix "boat keeps following old path".

**Documented trade observed:**
- "When I sent a different line, it tried following it without transiting to it" — this is the planned in-place-switch behavior (plan's "Redesign — in-place switch" decision with Roland 2026-05-26): re-sent line may be joined partway rather than from its start. Transit-to-start on different-line would require exit-SurveyLine and re-enter TransitAndSurveyLine (not in scope for #35).

**Pre-existing console noise unrelated to #35:**
- `/ben/hover_visualization` "more than one type associated" recorder warnings (two publishers, two message types on the same topic name; visible only to `ros2 bag`).
- CAMP process died with exit code -6 (SIGABRT) at 09:50:07 (during the test). Pre-existing CAMP stability issue; not from this BT XML edit (the change is BT-only).

### cmd_vel continuity (relevant to #28 / PR #40)
`/ben/cmd_vel_nav` shows 6 zero-cmd windows of 0.5–1.0 s, each coinciding with a `run_tasks` ABORT (operator restart) — `CancelFollowPath` → planner re-plan → new `FollowPath`. This is the line-transition stall #28 / PR #40 addresses, **not** caused by this change. The reactive `SetPathFromTask` path does not introduce a zero-cmd window — natural multi-line transitions within one `run_tasks` goal had no zero-cmd gap measurably above the regular 10 Hz cadence.

### Plan-review findings disposition
- **(must-fix) Name the actual wiring**: Done — autoremap chain + `survey_line_task` key documented in commit message + BT XML comment block.
- **(must-fix) Regression test or deferral**: **Deferred** to a follow-up (no BT-task-navigator harness exists yet; the test_dispatch_routing pattern from PR #37 could be the seed, but would require a mock `FollowPath` action server to verify the reactive update). The sim verification + source-reading carry the load here. Plan to add a unit test in a follow-up issue once the test infrastructure exists.
- **(suggestion) Step-3 wiring keys**: addressed in BT XML comment.
- **(suggestion) INFO log on path re-send**: not added (no `LogText` node in the tree; Nav2 already logs `Received a new path...` at the controller when `send_new_goal` runs).
- **(suggestion) Spike: positively confirm via goal-UUID/BT-log**: done via the `follow_path/_action/status` bag (UUID transitions recorded).
- **(suggestion) In-place-switch edge cases**: added as a Consequences note above; documented as the explicit operator-acknowledged trade.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-28 10:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: `feature/issue-35` at `<sha>` (TBD on commit)
**Mode**: pre-push (base `origin/jazzy`)
**Depth**: Standard-equivalent — small BT XML delta on a hot path; source-reading + sim evidence + plan-review approval already in place.
**Must-fix**: 0 | **Suggestions**: 1 (deferred regression test, see Implementation)

### Findings
- [ ] (suggestion, deferred) Add a BT-routing unit test that mocks `FollowPath` and asserts `{survey_path}` reflects the new task on `{survey_line_task}` swap without restarting `run_tasks`. Mirror the `test_dispatch_routing` pattern from PR #37. Deferred to a follow-up because: (a) no mock `FollowPath` infrastructure exists in this package yet; (b) the source-reading + sim evidence cover the contract; (c) the test would add ~150 LoC of fixture for a 1-line XML change. Reasonable as separate work.
