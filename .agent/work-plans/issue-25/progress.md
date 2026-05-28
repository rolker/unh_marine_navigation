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

## Open Questions Resolved + Merge with #35
**When**: 2026-05-27 09:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — decided with Roland

Resolved the four plan open questions and merged #25 with #35:

- **Q1 (observability)** → write a **structured** `TaskInformation.status`
  (`{outcome, reason, attempts}`) + `RCLCPP_ERROR`. It rides the heartbeat to **camp**,
  which renders mission status — so the existing field is the operator's eyes; no
  `DiagnosticStatus` publisher. Separate follow-up (camp repo): improve camp's
  mission-status display. No `.msg` change.
- **Q2 (test depth)** → node gtest **+** a minimal BT-XML **routing fixture**
  (matched-but-failed → `SetTaskFailed`; unmatched → catchall). No #8 dependency;
  full-tree integration still deferred to #8.
- **Q3 (scope across levels)** → mirror retry + `SetTaskFailed` at **all three** levels
  (`NavigatorSequence`, `RunSurveyAreaSubTasks`, `SurveyLineSetTask`). Nested levels gain
  skip-bad-line-and-continue instead of abandon-area-on-one-abort.
- **Q4 (#35 coupling)** → **merge #25 + #35** into this PR. The retry/`SetTaskFailed`
  machinery and #35's identity-gate FAILURE collide on the same signal (preemption vs
  execution failure); the seam belongs where the machinery lives. `feature/issue-25` /
  **PR #37** is canonical and closes both; PR #36 closed as superseded.

Plan rewritten to the combined scope (Parts A–D). Central risk: the Part C
preempt-vs-fail seam (a legitimate switch must not burn the retry budget or be flagged
`failed`) — pinned by the re-entry test fixture. `unh_marine_navigation#35` plan is
superseded by this combined plan.

## Plan Review
**Status**: complete
**When**: 2026-05-27 10:33 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — independent fresh-context sub-agent review (same agent identity, but no access to author reasoning)

**Plan**: `.agent/work-plans/issue-25/plan.md` at `ebb2da9`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/37
**Verdict**: changes-requested

Diagnosis, merge rationale, the Switch-vs-default structural fix, and principle/ADR
alignment are sound. But Part C (the central design artifact) was built on a wrong model
of the tree, and the status→operator path was misattributed. All findings below verified
against source before persisting.

### Findings
- [ ] (must-fix) Part C `SurveyLineTask` structure wrong: the survey `FollowPath` is two subtree levels down (`SurveyLineTask → TransitAndSurveyLine → SurveyLine`), behind a transit leg (`NavigateThroughWaypoints`). The plan's diagram places `FollowPath` directly under the identity gate — not implementable as drawn. — `run_tasks.xml:347,362,282`; `plan.md` Part C
- [ ] (must-fix) Identity gate can't halt a RUNNING `FollowPath` from inside `SurveyLineTask`'s memory `Sequence`; the reactive wrapper already exists inside `SurveyLine` (around `FollowPath`) — the gate must live there (and likely use `CancelAllNavigation` on re-latch). — `run_tasks.xml:282-302`
- [ ] (must-fix) Part C retry-budget concern under-characterized: real risks are no-halt/re-latch loop, transit-leg replay on re-entry, and `SetPathFromTask` (SyncAction) re-emitting the path every tick if placed above a reactive gate. Redesign against the real two-level nesting. — `set_path_from_task.cpp:60-71`
- [ ] (must-fix) Status→operator path misattributed: it is `task_navigator.cpp:116` (`taskMessages()` feedback) → `unh_marine_autonomy/.../camp_interface.py:listTasks` (`if len(task.status): ' status: ' + str(task.status)`), NOT `bt_types.cpp:245`. Writing `status` for the first time changes camp text TODAY — a cross-repo consequence the table reduces to "audit." (`.msg`-unchanged conclusion still holds.) — `camp_interface.py:94-95`
- [ ] (suggestion) "Structured status as a stable contract for camp" overstated — `camp_interface.py` renders `str(task.status)`; no consumer parses `{outcome,reason,attempts}` today.
- [ ] (suggestion) `Switch` is not reactive across ticks like the current `ReactiveFallback`; confirm the top-level `ReactiveSequence` re-tick of `UpdateCurrentTaskData` still drives re-selection on a mid-run type change.
- [ ] (suggestion) "Mirror at all three levels" is three *different* edits — nested levels use `_failureIf` + `RetryUntilSuccessful` already; not one repeated edit.
- [ ] (suggestion) gtest infra already exists (`test_set_controller_speed_resolve.cpp`) — "extend," not "add"; the re-entry fixture must model the real two-level `FollowPath` placement or it validates a tree that isn't shipped.
- [ ] (suggestion) `path_task_id`/`current_task_id` resolution across the three `SurveyLineTask` call sites (different id keys + `_autoremap`) is a correctness item, not just validation.

## Redesign + Un-merge from #35
**When**: 2026-05-27 11:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — decided with Roland

A clean-slate design pass (prompted by Roland asking whether a fresh design would be
simpler) reframed both fixes and reversed the merge:

- **#35 → in-place path switch.** Roland chose to relax the earlier "transit-to-start"
  decision. #35 becomes: re-derive `{survey_path}` reactively from `current_task` so Nav2
  `FollowPath` auto-resends on a path change (mechanism confirmed in `BtActionNode`
  `goal_updated_`/`send_new_goal`; the `FollowPathAction.cpp` path-comparison still needs a
  sim spike). No identity gate, no halt, no FAILURE → **decoupled from #25**. So the two
  are **un-merged**: #35 standalone on `feature/issue-35` / PR #36 (reopened); this PR (#37)
  closes #25 only. The earlier preempt-vs-fail seam (the Part C risk) is gone.
- **#25 → Switch + marine recovery wrapper.** Recovery investigation: no `RecoveryNode`
  is wired today (FollowPath ABORT is unhandled — the bug); `behavior_server` configures
  recoveries but nothing invokes them, and `spin`/`backup` are wrong for a boat. Roland
  chose to keep the retry intent, done properly as a Nav2 `RecoveryNode` invoking a marine
  recovery (`wait`/`hover` TBD), centralized at the shared `SurveyLine` node (one edit, not
  three). On exhausted recovery, `SetTaskFailed` records `status` + advances; record/advance
  is wired at the top `Switch` case + the two nested loops (skip-and-continue).
- **Corrected from the prior plan:** the `status`→operator path is
  `task_navigator.cpp:116` → `camp_interface.py:listTasks` (`unh_marine_autonomy`), NOT
  `bt_types.cpp`; writing `status` changes camp text today. The survey `FollowPath` is two
  subtree levels down in `SurveyLine` (fixes the review's must-fix structural error).

Plan rewritten to the standalone #25 scope.

## Plan Review (post-redesign, standalone #25)
**Status**: complete
**When**: 2026-05-27 12:04 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — independent fresh-context sub-agent review

**Plan**: `.agent/work-plans/issue-25/plan.md` at `3c51499`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/37
**Verdict**: changes-requested (ready-with-fixes — plan refinements, not approach rework)

Verified correct against source: Switch-with-default structural fix; stock BT.CPP `Switch`
+ stock nav2 `RecoveryNode`/`Wait` availability (no plugin_lib additions); no `.msg` change
(`TaskInformation.msg:32`); cross-repo camp path (`task_navigator.cpp:116` →
`camp_interface.py:94-95`); ADR-0008/0002 alignment.

### Findings
- [ ] (must-fix) Nested-loop `SetTaskFailed` collides with the `_failureIf` routing gates in `RunSurveyAreaSubTasks` (`run_tasks.xml:205,210`): a blanket failure-fallback there would mark a *type-mismatched* (legitimately routed) sub-task as `failed`, re-conflating unmatched-vs-failed one level down. The nested edits must preserve the matched-vs-unmatched distinction — design against the real XML.
- [ ] (must-fix) `RecoveryNode`-at-`SurveyLine` semantics under-specified (`run_tasks.xml:282-302`): (a) wrap only `FollowPath` vs the whole `ReactiveSequence` — retrying re-follows the *full* `{survey_path}`, re-covering surveyed trackline; (b) `CancelAllNavigation` + a `Hover` recovery + retry need cancel/start ordering; (c) a persistent stuck condition just re-trips the 10 s progress checker → retries exhaust → `SetTaskFailed` (intended terminal — state that recovery is for transient trips only).
- [ ] (suggestion) Recovery is invoked via a BT *action node* (`Wait`, or the repo's custom `Hover` at `bt_register_nodes.cpp:52`), not the `behavior_server` plugin list — name the BT node in Open Q1.
- [ ] (suggestion) The routing fixture must exercise a mid-run `current_task_type` change (Switch reactivity) AND a nested-loop failed-line → records-failed-and-loop-continues assertion (where Finding-1's bug lives).
- [ ] (suggestion) Name a second-consumer search for the RunTasks-feedback `tasks[].status` beyond `camp_interface.py`; "structured status" is written but only `str()`-rendered today (not a consumed contract).

## Implementation (in progress)
**When**: 2026-05-27 13:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Landed on `feature/issue-25` (PR #37):**
- `d94d240` — `SetTaskFailed` node (+ registration, CMakeLists, unit test). Builds clean;
  3 gtest cases pass (records failed status; omits unset optional fields; contrast with
  `SetTaskDone` leaving status empty).
- `7ec798c` — top-level `NavigatorSequence` dispatch: `ReactiveFallback` → `Switch5` on
  `current_task_type`; default child = the existing `SetTaskDone` catchall; each matched
  case wrapped `Fallback[<task>, SetTaskFailed]`. XML well-formed (xmllint). Generalizes
  the fix to all task types (the silent-mark-done bug applied to hover/goto too).

**Env note:** main `core_ws` rebuilt via `make build` (core layer ✅, so the stale-install
codegen gotcha is cleared). `make build` overall fails at the **ui layer** (`image_warper`
missing `catkin`) — unrelated to #25, pre-existing; flagged to Roland.

**Remaining:**
- `RecoveryNode[FollowPath, Wait]` at the shared `SurveyLine` node.
- Nested-loop Switch-ify (`RunSurveyAreaSubTasks`, `SurveyLineSetTask`) preserving the
  matched-vs-unmatched distinction (review must-fix) — i.e. same Switch+default shape, not
  a blanket failure-fallback.
- Routing fixture test (matched-failed → SetTaskFailed; unmatched → catchall; mid-run type
  change; nested skip-and-continue).
- **Runtime tree-load verification needs a sim session** — no standalone harness loads the
  full `run_tasks.xml` (it references nav2 + marine nodes); xmllint only confirms
  well-formedness. This dovetails with the #35 Nav2-resend spike (same sim).

## Implementation complete (code) — 2026-05-27 ~13:40 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

All #25 code landed on `feature/issue-25` (PR #37), built clean, **17 gtests pass**:
- `d94d240` SetTaskFailed node + unit test.
- `7ec798c` top-level NavigatorSequence Switch dispatch.
- `57d0410` merge of `origin/jazzy` (incorporated #33; 1 trivial CMakeLists conflict resolved as union).
- `2e607e6` RecoveryNode[FollowPath, Wait] at SurveyLine + nested Switch-ify (RunSurveyAreaSubTasks Switch + SurveyLineSetTask Fallback) — skip-and-continue, no unmatched-vs-failed conflation (review must-fix resolved).
- `fa4043d` routing fixture (test_dispatch_routing) + SetTaskFailed Groot model entry.

**Verified:** xmllint well-formed; builds clean (with the new-node LD_LIBRARY_PATH codegen
workaround — needed until #25 merges); `test_dispatch_routing` proves matched-but-failed →
SetTaskFailed and unmatched → clean-done catchall at runtime with stub leaves.

**Lint note (not a regression):** unfiltered `colcon test` shows ~396 lint failures
(cpplint 289/289, copyright 53/54, uncrustify 48/54) — pre-existing package-wide debt
(#33 merged through it; package has no copyright headers / never passed cpplint). New files
follow the package's no-header house style. Enforced gates are pre-commit + CI, which pass.
Whole-package lint cleanup would be a separate issue.

**Remaining before PR-ready:** (1) runtime full-tree load verification in sim (no standalone
harness loads run_tasks.xml; dovetails with the #35 Nav2-resend spike); (2) /review-code
pre-push; (3) flip PR #37 out of draft after sim verification.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 14:10 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved (1 must-fix found + fixed)

**Branch**: feature/issue-25 at `300fad7`
**Mode**: pre-push (base origin/jazzy)
**Depth**: Deep (reason: mission-critical autonomy BT control-flow — Switch halt, RecoveryNode retry/cancel, failure routing)
**Specialists**: Claude Adversarial (Deep) ✓; Copilot Adversarial skipped (CLI auth error mid-run — needs `copilot` re-login); Static/Governance/Plan-Drift inline.
**Must-fix**: 1 (fixed) | **Suggestions**: 3

### Findings
- [x] (must-fix) Null `current_task` deref in SetTaskDone/SetTaskFailed default catchall on the clear-mission path — guarded in `300fad7` (+ null-task test). `set_task_{done,failed}.cpp`
- [ ] (suggestion) RecoveryNode retry re-follows the full `{survey_path}`, re-covering surveyed trackline — confirm acceptable in sim. `run_tasks.xml` SurveyLine
- [ ] (suggestion) Stacked retries: RetryUntilSuccessful(3) × RecoveryNode(3) = up to 9 FollowPath attempts (with 5s Waits) on a persistently-stuck nested sub-line before skip — confirm the retry budget / time-to-skip is intended, or reduce a count. `run_tasks.xml` nested loops
- [ ] (suggestion) `SetTaskFailed` `attempts` port is never wired (BT can't read RecoveryNode/Retry counters) — reserved for future use; drop or document. `set_task_failed.cpp`

**Verified-correct (adversarial):** Switch halt on mid-run type change preserves clear→resend; Fallback+SetTaskFailed advances without infinite loop / re-exec; nested skip-and-continue terminates via AllTasksDone; all nav2/BT nodes resolve (no tree-load failure from node names); `wait` server configured; `_autoremap`/task-key wiring correct; tests faithfully model routing.

**Governance:** Principles — transparency Pass (failures recorded+logged, no silent done), test-what-breaks Pass (node+routing fixtures; full-tree → #8), only-what's-needed Pass. ADR-0008/0002 Pass. Consequence: writing `TaskInformation.status` surfaces in camp via `unh_marine_autonomy/.../camp_interface.py` (cross-repo, no `.msg` change) — second-consumer grep still an open pre-merge item.
**Plan drift:** Implementation matches plan (Switch + SetTaskFailed + RecoveryNode[Wait] + nested Switch-ify + routing fixture). The 3×3 stacked-retry interaction (suggestion 3) is an emergent gap the plan didn't call out.
**Static:** package-wide pre-existing lint debt (cpplint/copyright/uncrustify); new files follow house style; pre-commit (enforced) passes. No new actionable static findings.

## Integrated Review
**Status**: complete
**When**: 2026-05-27 17:38 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #37 at `79b7a24`
**Sources**: 2 (Copilot R1 @ `79b7a24`, Local Review (Pre-Push) @ `300fad7` — same code state)
**Cross-source confirmations**: 2
**CI**: copilot-pull-request-reviewer success; no build/test CI wired on this repo

### Findings
- [x] (cross-confirmed, needs-decision) Stacked retries: nested survey-area (`run_tasks.xml:245`) and survey-line-set (`:404`) loops wrap `SurveyLineTask` in `RetryUntilSuccessful num_attempts=3`, while `SurveyLine` already wraps `FollowPath` in `RecoveryNode number_of_retries=3` + 5s `Wait` (`:358-370`) → up to 9 attempts (~minutes dwell) before `SetTaskFailed` on a stuck nested sub-line; top-level `survey_line` (`:178-184`) caps at 3 (no outer retry) — inconsistent. **Resolved in `cba7f49`**: dropped the outer `RetryUntilSuccessful` at all three nested sites (survey-area survey_line + survey_line_set cases, line-set sub-line loop) so they mirror the top-level `Fallback[<task>, SetTaskFailed]` shape; recovery centralized in `SurveyLine`'s `RecoveryNode(3)`, time-to-skip now capped at 3 FollowPath attempts uniformly. Operator can still tune the budget if a different cap is wanted. — `run_tasks.xml` (Copilot lines 269, 418)
- [x] (cross-confirmed, suggestion) Dead `attempts` port: declared+read but never wired at any of 8 call sites; BT can't read RecoveryNode/Retry counters so it's always 0 and `status["attempts"]` always omitted. **Resolved in `cba7f49`**: removed the port (declaration, read, conditional status write, TreeNodesModel entry) and the `attempts` test assertions (renamed `OmitsUnsetReasonAndAttempts` → `OmitsUnsetReason`). — `set_task_failed.cpp` / `run_tasks.xml` TreeNodesModel
- [ ] (carried, Local Review) RecoveryNode retry re-follows the full `{survey_path}`, re-covering surveyed trackline — confirm acceptable in sim. — `run_tasks.xml` SurveyLine

### Fixes applied (round 1)
**When**: 2026-05-27 19:43 -04:00
Both cross-confirmed findings resolved in `cba7f49`. Verified: `marine_nav_behavior_tree` gtests pass (`test_set_task_failed` 4/4, `test_dispatch_routing` 3/3); `run_tasks.xml` well-formed (xmllint). Build note: the build-time nodes-XML generator fails against the **stale `jazzy` underlay** install (its `_bt_plugins.so` predates `SetTaskFailed`, so the loader shadows the fresh lib) — environmental drift, not a code defect; built/tested by prepending the worktree build dir to `LD_LIBRARY_PATH`. Resolves once #25 merges and the underlay is rebuilt. Remaining: sim verification of the re-coverage-on-retry item before field use.

### Resolved (prior round)
- (must-fix, Local Review @ `300fad7`) Null `current_task` deref in SetTaskDone/SetTaskFailed default catchall — fixed in `300fad7` (guard + null-task test); Copilot did not re-flag.

### False positives
- None. All three Copilot inline comments are valid (its two `run_tasks.xml` comments are the same finding at lines 269 and 418).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 21:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: feature/issue-25 at `d592d18`
**Mode**: pre-push (base origin/jazzy)
**Depth**: Standard (small delta on mission-critical BT control-flow)
**Scope**: round-1 review-fix delta (`cba7f49` + comment reword `d592d18`)
**Specialists**: Claude Adversarial (fresh-context) ✓; Copilot Adversarial ✓ (cross-model); Static covered by colcon ament-lint + pre-commit.
**Must-fix**: 0 | **Suggestions**: 1 (fixed)

### Findings
- [x] (suggestion, cross-confirmed Claude+Copilot) Comments said failures route to SetTaskFailed "after retries"/"after RecoveryNode exhausts"; the Fallback now catches ALL line-subtree failures (transit/path-setup/TF), not just FollowPath retry-exhaustion. Broadened both comments in `d592d18`. — `run_tasks.xml:237,398`

**Verified-correct (both reviewers, independent):** port removal complete (no lingering `attempts` ref / no dangling getInput); retry removal structurally valid (all three nested sites mirror the unchanged top-level Fallback[Sequence,SetTaskFailed]; only the unrelated `:114` num_attempts=20 transit retry remains); no non-termination (KeepRunningUntilFailure bounded by AllTasksDoneCondition; both SetTaskDone/SetTaskFailed call setDone()); tests track the change.
**Note:** accepted trade-off — non-FollowPath transient failures on nested lines now skip after one attempt (consistent with top-level dispatch); retrying the transit leg would need its own RecoveryNode in TransitAndSurveyLine, not a blanket outer retry.

## Integrated Review
**Status**: complete
**When**: 2026-05-27 22:04 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #37 at `9c0dcc5`
**Sources**: Copilot R2 @ `9c0dcc5` (current) + Copilot R1 @ `79b7a24` (stale, prior round) + local timeline
**Cross-source confirmations**: 0 (new finding is Copilot-only; prior local pre-push reviews — incl. both adversarial passes — missed it)
**CI**: copilot-pull-request-reviewer success; no build/test CI wired

### Findings
- [ ] (must-fix, Copilot R2) `SurveyLineSetTask` SubTree call in the survey-area loop (`run_tasks.xml:262-265`) lacks the explicit `current_task_type="{current_survey_area_task_type}"` remap its sibling case_1 has (`:254`). Verified autoremap chain: `SurveyAreaTask` gated on `current_task_type=='survey_area'` (`:316`) → autoremaps into `RunSurveyAreaSubTasks`; `UpdateCurrentTaskData` writes the sub-task type to `current_survey_area_task_type` (`:228`), NOT `current_task_type`; so `SurveyLineSetTask.current_task_type` autoremaps to the inherited `"survey_area"`, its entry guard `ScriptCondition current_task_type=='survey_line_set'` (`:375`) fails → FAILURE → Fallback → SetTaskFailed. Net: every survey_line_set sub-task within a survey_area is falsely recorded failed + skipped. Re-introduces the unmatched-vs-failed conflation #25 removes, one level down. Fix: add the explicit remap mirroring case_1. — `run_tasks.xml:262-265`

### Resolved (R1, stale @ `79b7a24` — prior round)
- (Copilot R1) Dead `attempts` port (`set_task_failed.h:21`) — removed in `cba7f49`.
- (Copilot R1) Stacked retries survey-area (`run_tasks.xml:269`) + line-set (`:416`) — outer RetryUntilSuccessful removed in `cba7f49`.

### False positives
- None.
