---
issue: 28
---

# Issue #28 — Survey line-transition stall: uncommanded gap trips FCU 3s GUIDED watchdog

## Issue Review
**Status**: complete
**When**: 2026-05-27 13:02 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #28
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/28#issuecomment-4557000755
**Scope verdict**: well-scoped

### Actions
- [ ] Sequence after #35 (PR #36): let its "does FollowPath resend on a RUNNING path-input change?" spike resolve first; it answers #28's hot-swap feasibility. Re-measure the transition gap on #35's branch before committing to a separate #28 fix.
- [ ] In plan-task, decide the lever explicitly: (a) hot-swap path (no cancel/zero, depends on the spike) vs (b) bound the gap < 3 s (raise the transit-replan `RateController hz` + drop the pre-emptive `CancelAllNavigation`/zero); quantify the margin against the FCU 3 s watchdog.
- [ ] Land a command-continuity test with the fix (timing-dependent bug → deterministic repro on the `unh_echoboats_project11#186` dry-run/sim bed; assert no >Ns zero-command window across a line transition).
- [ ] Coordinate `run_tasks.xml` edits with #25 (PR #37) and verify no regression with #23 (stale per-line goal `header.stamp`); refresh the embedded `TreeNodesModel` if node usage changes.
- [ ] Honor "keep way, not zero" — note velocity_smoother/cmd_vel filters are disabled on BizzyBoat, so controller-output continuity is the whole fix (no smoothing layer to mask a gap).

## Plan Authored
**Status**: complete
**When**: 2026-05-27 13:50 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-28/plan.md` at `4e20532`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/40 (`[PLAN]` prefix)
**Phases**: single PR (BT-only + sim continuity test)
**Scope**: Option A only (band-aid); B2 look-ahead pre-plan + coverage-planner redesign excluded by user direction.

### Open questions
- [x] Q1 (gating): does `follow_path` preempt a RUNNING goal without a zero-command blip? — answered via the #35/#36 spike (upstream `follow_path_action.cpp` confirms `on_wait_for_result` reads the `path` port every tick and sets `goal_updated_` on change). Preemption is seamless once the new goal lands.
- [x] Q2: target transit `RateController hz` + planner retry cap vs the 3 s margin — picked `hz=2.0` (re-plan every 0.5 s, 6x under the 3 s margin) and `num_attempts=4` (down from 20).
- [x] Q3: split a hover-only exit out of the shared `CancelAllNavigation` — split chosen (new `ExitHoverForNav` subtree; `CancelAllNavigation` left intact for `GotoPose`).

## Implementation
**Status**: complete (code) — sim repro deferred per user direction (June 4 freeze pressure)
**When**: 2026-05-28 10:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Branch**: `feature/issue-28`
**Diff scope**: single-file BT XML edit — `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` (+33 / -4 lines).

### Changes
1. **New `ExitHoverForNav` subtree** (`run_tasks.xml:18-29`): `ForceSuccess[CancelHover]` only — does **not** cancel the live `FollowPath`. Used to leave any active hover before nav without dropping the controller output. Doc-comment in XML explains why and points to #28.
2. **`NavigateThroughWaypoints` entry (`:87` → now `:107`) uses `ExitHoverForNav`** instead of `CancelAllNavigation`. The next `FollowPath` goal preempts the active one at the `follow_path` server (#35/#36 spike confirms `goal_updated_` mechanism). No uncommanded zero-cmd window from a pre-emptive cancel.
3. **`SurveyLine` entry (`:293` → now `:322`) uses `ExitHoverForNav`** for the same reason — the survey `FollowPath` goal preempts the transit `FollowPath` cleanly.
4. **`GotoPose` (`:18-29`, unchanged) still calls `CancelAllNavigation`** — operator hard-stop semantics for goto are correct as-was.
5. **`RateController hz="0.5" → "2.0"`** on transit re-plan cadence (`:103` → now `:127`). 0.5 s between re-plans is well under the FCU 3 s GUIDED watchdog margin.
6. **`RetryUntilSuccessful num_attempts="20" → "4"`** on `ComputePathThroughPoses` (`:114` → now `:138`). Bounds worst-case planner-stall to roughly `4 × server_timeout_per_attempt`, instead of 20×. On exhaustion, the BT fails the dispatch (to `SetTaskFailed` post-#37, or to the catchall pre-#37) — loud rather than silent stall.

### Sim repro — DEFERRED
The deterministic continuity repro (`unh_echoboats_project11#186` dry-run/sim bed, assert <3 s zero-cmd window) is **not in this PR**. Reason: June 4 deployment freeze pressure; full sim wiring on `marine_simulation simulator_launch.py` plus multi-line scripted mission upload is a separate scope item. The mechanism is verified from upstream source (Nav2 `FollowPathAction::on_wait_for_result` auto-resends — same spike result that backs PR #36).

The bag captured during PR #36 verification (`.scratchpad/issue-35-sim/midline_switch_0.mcap` on dev workstation) already shows the **pre-#28 behavior**: 6 zero-cmd windows of 0.5–1.0 s on `/ben/cmd_vel_nav`, each at a `run_tasks` ABORT or natural line transition. With this PR's `CancelFollowPath` removal at the line-entry callsites, those windows should shrink toward the controller's normal cadence interval. A follow-up sim run will confirm.

### Follow-up
- Sim continuity repro — file as follow-up issue once #40 is review-cleared. Same harness will be reused for #28 retests and future watchdog-related regressions.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-28 10:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: `feature/issue-28` at `<sha>` (TBD on commit)
**Mode**: pre-push (base `origin/jazzy`)
**Depth**: Standard-equivalent — small BT XML delta on a hot path; mechanism evidenced by PR #36 source-spike + bag.
**Must-fix**: 0 | **Suggestions**: 1 (deferred sim repro, see Implementation)

### Findings
- [ ] (suggestion, deferred) Deterministic sim continuity repro (`<3 s` zero-cmd window across line transitions) is not in this PR; deferred per June 4 freeze pressure. To be filed as a follow-up issue and run before the next field deployment.
