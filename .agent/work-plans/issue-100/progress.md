---
issue: 100
---

# Issue #100 — CrabbingPathFollower: post-schedule crab angle exceeds ±90°, causing sail-away equilibrium and speed inflation

## Issue Review
**Status**: complete
**When**: 2026-07-23 12:00 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #100
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Bug in `CrabbingPathFollower::computeVelocityCommands`: the PID is clamped to
±90° internally (`initialize_from_args(…, 90.0, -90.0, …)` — verified at
`crabbing_path_follower.cpp:213`), but `gainScheduleScale` multiplies the output
**after** that clamp (`path_geometry.hpp:188-200`). At Bizzy's tune
(`gain_ref_speed=1.8`, `target_speed=1.5 m/s`) the factor is 1.2 → crab = ±108°,
past perpendicular. Field-replicated from bag `2026-07-21T17-26-41`.

Two independently harmful consequences verified in code:
1. `target_heading = base_heading + crab_angle` (`cpp:976`) — with |crab| > 90°
   the along-track component is negative; the heading loop converges to the wrong
   course and holds it indefinitely (sail-away equilibrium).
2. `cos_crab = max(cos(crab_angle), 0.5)` (`cpp:1053`) — floor activates at
   |crab| ≥ 60°, commanding 2× target surge. Observed 3.09 m/s vs 1.5 default.

Fix direction is clear: clamp crab to ±(90° − ε) **after** gain scheduling,
before the heading and surge calculations consume it. Issue correctly notes that
this also restores `cos(crab) > 0` and bounds the speed inflation to the intended
`1/cos` compensation.

### Scope Assessment

**Well-scoped?** Yes — the mechanical fix (clamp post-schedule) is a one-line
change with a known parameter. Unit tests are explicitly specified (railed PID ×
factors {1.0, 1.2, 1.8} → assert |crab| < 90°, along-track ≥ 0, surge bound).
The "acquisition mode" suggestion is correctly deferred as a separate concern and
does not need to be part of this PR.

**Right repo?** Yes — bug is entirely in
`marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` in
`unh_marine_navigation`.

**Dependencies?** No blocking dependencies. Related to #76 (gain schedule
introduction) and field bag from #381 (2026-07-21 Massabesic RCA). Issue #99
(segment-cursor fix) merged and no conflict expected.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Bug causes silent wrong-course convergence; fix restores predictable bounded behavior. Fix direction fully documented with field numerics. |
| Enforcement over documentation | OK | Issue mandates unit tests; existing `test_gain_schedule.cpp` provides the test scaffold. Tests are the enforcement. |
| Capture decisions, not just implementations | Watch | The choice of ε (5°–10°), whether to make the post-schedule clamp a configurable parameter, and the anti-windup interaction are design decisions worth capturing in plan.md or a comment. |
| A change includes its consequences | OK | Issue explicitly lists downstream consumers (`turnSpeedFactor`, `cos_crab`) and notes they benefit from the fix. Test requirements specified. `test_crabbing_control.cpp` end-to-end path should also cover the fixed invariant. |
| Only what's needed | OK | Fix is minimal; acquisition mode deferred. |
| Improve incrementally | OK | Single-PR scope, clear before/after invariant. |
| Test what breaks | OK | Explicit test matrix specified. The railed-PID × schedule-factor combination is exactly the failure mode that broke field operations — high priority test case. |
| Workspace vs. project separation | OK | All changes in project repo. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Already in worktree `issue-unh_marine_navigation-100`. ✓ |
| ADR-0008 — ROS 2 conventions | Yes | Modifying a `nav2_core::Controller` plugin; no convention conflict expected from a clamp-order fix. |
| ADR-0013 — progress.md vocabulary | Yes | Writing `## Issue Review` entry. ✓ |
| ADR-0018 — Local-first CI | Yes | Project-repo change; `ci_local.sh` verification required before merge. |

### Consequences

- The post-schedule clamp shrinks the input domain of `turnSpeedFactor` and
  `cos_crab` to |crab| < 90° — explicitly noted in the issue as a benefit.
- If the implementer adds a configurable `pid.max_crab_deg` parameter for the
  post-schedule clamp, parameter descriptors and documentation need updating.
- `test_gain_schedule.cpp` and/or `test_crabbing_control.cpp` need new test cases
  covering the fix invariant.
- The low-speed edge case (v → `gain_v_min`) can amplify past ±162° at the current
  default `gain_v_min=0.5` — the fix must cover this case too (as the issue notes).

### Actions
- [x] Decide: hardcode ε (e.g. 5°) or add `pid.max_crab_deg` configurable parameter — capture the choice in plan.md with rationale.
  - **Operator decision (2026-07-23 checkpoint)**: hardcode ε = 5° (post-schedule clamp ±85°). |crab| < 90° is a correctness invariant, not a tuning knob — a configurable limit could be set past perpendicular under field pressure and reintroduce the sail-away. Matches the existing hardcoded ±90° PID init. Capture rationale in plan.md.
- [x] Verify anti-windup interaction: if internal PID clamp stays at ±90° and post-schedule clamp is tighter, the PID may rail more often against its internal limit. Document whether this is acceptable or the PID internal clamp should also be adjusted.
  - **Operator decision (2026-07-23 checkpoint)**: keep the PID internal clamp at ±90° unchanged; the post-schedule clamp only trims the scheduled output. Windup stays bounded by the existing internal clamp. Plan must document the interaction and add a test asserting no windup growth while railed.
- [ ] Add unit tests: railed PID × schedule factors {1.0, 1.2, 1.8} and low-speed edge (v = gain_v_min) → assert |post-schedule crab| < 90°, along-track component ≥ 0, `linear.x ≤ target_speed / cos(clamp_limit)`.
- [ ] Check `test_crabbing_control.cpp` for an end-to-end scenario that validates the sail-away elimination.

## Plan Authored
**Status**: complete
**When**: 2026-07-23 14:30 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-100/plan.md` at `2213b57`
**Branch**: feature/issue-100 at `2213b57`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-23 08:43 -04:00
**By**: Claude Code Agent (Claude Opus)
<!-- Independent review: authored by a Sonnet dispatch; this is a separate fresh-context Opus sub-agent. The shared "Claude Code Agent" name collides but the review is not an author self-review. -->

**Plan**: `.agent/work-plans/issue-100/plan.md` at `2213b57`
**PR**: PR-less (`--issue` mode; `gh` unauthenticated — issue content sourced from the committed `## Issue Review` entry)
**Verdict**: changes-requested

### Findings
- [x] (must-fix) Clamp is inline in `computeVelocityCommands` behind an anonymous-namespace constant, but tests target `test_gain_schedule.cpp` which only reaches `path_geometry.hpp` pure functions — the clamp is unreachable, no `computeVelocityCommands` harness exists. Extract the clamp as an `inline` helper in `path_geometry.hpp` (mirroring `gainScheduleScale`) so it is unit-testable. — `plan.md:34`, `plan.md:46`
- [x] (must-fix) Surge-bound math wrong: `cos_crab = max(cos(crab), 0.5)` (`crabbing_path_follower.cpp:1054`) floors the divisor at 0.5, so `linear.x ≤ 2× target_speed` for any crab angle (before and after the fix). The "≈11.5×" consequence and the `linear.x ≤ target_speed/cos(85°)` assertion are both wrong/trivially-true; assert `linear.x ≤ 2*target_speed` and reframe the clamp as sail-away fix, not surge reduction. — `plan.md:52`, `plan.md:89`
- [x] (suggestion) Review-issue action to check `test_crabbing_control.cpp` for an end-to-end sail-away scenario is unaddressed; no such harness exists there today — resolve explicitly (cover-at-helper-level note or add a test). — `progress.md:92`
- [x] (suggestion) Relabel test factors `{1.0, 1.2, 1.8}` → `{1.0, 1.2, 3.6}` (1.8 is a `gain_ref_speed`, not a factor; at v=0.5 the factor is 3.6) and sync ~1-line-drifted source citations. — `plan.md:50`, `plan.md:60`
