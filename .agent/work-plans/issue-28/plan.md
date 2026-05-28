# Plan: Survey line-transition stall — uncommanded gap trips FCU 3s GUIDED watchdog

## Issue

https://github.com/rolker/unh_marine_navigation/issues/28

## Scope

**Option A only** (the narrow band-aid): keep a valid setpoint flowing through the
line-transition replan so the FCU's 3 s GUIDED watchdog never trips, **without** streaming a
literal zero (which brakes). Explicitly **out of scope**: B2 look-ahead pre-planning and the
continuous-coverage / coverage-planner redesign (`#29`/`#30`). Per the review-issue discussion,
A *bounds/masks* the latency; it does not remove it from the critical path — that's B2.

## Context

`TransitAndSurveyLine` = `Sequence[ GetSubPath, NavigateThroughWaypoints (transit),
SurveyLine ]`. **Two** `CancelFollowPath`s fire per line→line handoff, each via the shared
`CancelAllNavigation` subtree (`run_tasks.xml:4`):
1. **L87** — entering `NavigateThroughWaypoints`: cancels the just-finished previous line's
   FollowPath, then `ClearPath` → `ComputePathThroughPoses` (`RateController hz=.5`,
   `RetryUntilSuccessful num_attempts=20`) → `FollowPath(transit)`.
2. **L293** — entering `SurveyLine`: cancels the transit FollowPath *that step 1 just started*,
   then `FollowPath(survey_path)`.

Each `CancelFollowPath` idles the `follow_path` server → `CrabbingPathFollower` returns a zero
`Twist`. The uncommanded window (cancel → next FollowPath's first command, incl. planner compute)
is variable; the 2026-05-22 line2→line3 case hit ~4 s and tripped the watchdog
(`unh_echoboats_project11#169`). `CancelAllNavigation` is **shared** with `GotoPose` (L20), so it
can't be edited blindly.

## Approach

**Lever 1 (primary) — stop pre-emptively idling the controller on the survey path.**
The next `FollowPath` goal preempts the active one at the `follow_path` server, so the leading
`CancelFollowPath` is what *creates* the zero window, not what's required to switch paths. Keep
the boat commanded on the old path until the new goal swaps in.
- Introduce a hover-only exit subtree (e.g. `ExitHoverForNav`: the existing `CancelHover`
  `ForceSuccess` **without** `CancelFollowPath`) and use it in `NavigateThroughWaypoints` (L87)
  and `SurveyLine` (L293). Leave the full `CancelAllNavigation` for `GotoPose` unchanged.
- `CancelHover` stays — leaving an active hover before nav is still required.

**Lever 2 (bound) — keep the worst case under the watchdog margin.**
- Raise `NavigateThroughWaypoints`'s `RateController hz=.5` so re-plan/refresh cadence is well
  under 3 s (target TBD — see Open Questions; likely ~2 Hz).
- Cap `RetryUntilSuccessful num_attempts=20` on `ComputePathThroughPoses` so a stuck planner
  can't stack retries past the margin; on exhaustion, fail loudly rather than stall silently.

**Verify (timing-dependent bug ⇒ deterministic repro).** Reproduce the transition on the
`unh_echoboats_project11#186` dry-run/sim bed and assert the `autonomous_cmd_vel` zero-command
window across a line transition stays under a fixed bound (< 3 s, with margin). Land the repro
with the fix.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Add `ExitHoverForNav` subtree (CancelHover only); use it at L87 + L293 in place of `CancelAllNavigation`; raise the transit `RateController hz`; cap the `ComputePathThroughPoses` retry count. Refresh the embedded `TreeNodesModel` if node usage changes. |
| (sim/test) | Line-transition command-continuity repro on the `#186` dry-run bed (assert bounded zero-command window). |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Band-aid scoped to the watchdog trip; redesign (B2) explicitly excluded. |
| Test what breaks | Deterministic continuity repro lands with the fix; the bug is timing-dependent so a one-off field observation isn't enough. |
| A change includes its consequences | `CancelAllNavigation` is shared — split rather than mutate, so `GotoPose` is unaffected; coordinate `run_tasks.xml` edits with `#35`/`#25`. |
| Improve incrementally | Faithful to the existing Nav2 BT; no controller rewrite. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — Follow ROS 2 conventions | Yes | Relies on Nav2's documented FollowPath goal-preemption (new goal replaces active) rather than a bespoke setpoint streamer; if preemption isn't seamless (Open Q1), Lever 2's bounding is the idiomatic fallback. |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `CancelAllNavigation` usage on the survey path | Verify `GotoPose` (operator goto) still cancels FollowPath as before — that's why we split, not mutate | Yes |
| `run_tasks.xml` BT structure | Embedded `TreeNodesModel`; coordinate with `#35` (SurveyLine reactive path) + `#25` (failure routing) on the same file | Yes |

## Open Questions

- [ ] **Q1 (gating — the `#35` spike):** does the `follow_path` server preempt a RUNNING goal with a new path *without* a zero-command blip? If yes, Lever 1 alone eliminates the gap. If no, Lever 1 shrinks it and Lever 2 must bound the residual. Implementation should follow / co-run `#36`'s spike rather than duplicate it.
- [ ] **Q2:** target `RateController hz` and retry cap — pick numbers against the measured worst-case compute time vs the 3 s margin.
- [ ] **Q3:** split (`ExitHoverForNav`) vs parameterize the shared cancel — confirm the split is the lower-risk choice for `GotoPose`.

## Estimated Scope

Single PR in `unh_marine_navigation` (BT-only + a sim continuity test). Implementation gated on the
`#35`/`#36` spike (Q1); sim-verify before relying on it on-water.
