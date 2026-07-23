# Plan: CrabbingPathFollower: post-schedule crab angle exceeds ±90°, causing sail-away equilibrium and speed inflation

## Issue

https://github.com/rolker/unh_marine_navigation/issues/100

## Context

In `CrabbingPathFollower::computeVelocityCommands`, the PID is internally clamped at
±90° (`initialize_from_args(…, 90.0, -90.0, …)` — `crabbing_path_follower.cpp:213`),
but `gainScheduleScale` multiplies the output **after** that clamp
(`crabbing_path_follower.cpp:908–910`). At Bizzy's tune (`gain_ref_speed=1.8`,
`target_speed=1.5 m/s`) the factor is 1.2 → |crab| reaches 108°. At the low-speed
floor (`v=gain_v_min=0.5`) the factor reaches 3.6 → |crab| up to 162°.

Two consequences when |crab| > 90°:
1. **Sail-away**: `target_heading = base_heading + crab_angle` (`cpp:976`) — the
   along-track component goes negative; the heading loop converges to the wrong
   course and holds it indefinitely.
2. **Speed inflation**: `cos_crab = max(cos(crab), 0.5)` (`cpp:1053`) — floor
   activates at |crab| ≥ 60°, commanding 2× target speed (3.09 m/s observed vs 1.5 default).

**Design decisions (from Issue Review checkpoint 2026-07-23):**
- ε = 5° (hardcoded); post-schedule clamp is ±85°. |crab| < 90° is a correctness
  invariant, not a tuning knob — a configurable limit could be set past perpendicular
  under field pressure and re-introduce sail-away. Matches the existing hardcoded ±90°
  PID init.
- Keep the PID internal clamp at ±90° unchanged. The post-schedule clamp only trims
  the scheduled output; windup stays bounded by the existing conditional-integration
  anti-windup.

## Approach

1. **Add `kPostScheduleCrabClampDeg` constant in `crabbing_path_follower.cpp`** (anonymous
   namespace, alongside the existing `kTunables` table) — 85.0 degrees.

2. **Clamp `crab_angle` after gain scheduling** — one line after `cpp:910`:
   ```cpp
   crab_angle = AngleDegrees(std::clamp(crab_angle.value(), -kPostScheduleCrabClampDeg, kPostScheduleCrabClampDeg));
   ```
   Add a comment documenting: why ε=5° (correctness invariant, not a knob), the
   anti-windup interaction (internal PID clamp stays at ±90°; the post-schedule
   trim has no windup effect since the integral only accumulates within the PID),
   and the field evidence (bag `2026-07-21T17-26-41`, issue #100).

3. **Add post-schedule clamp tests in `test_gain_schedule.cpp`** — new test fixture
   `PostScheduleClamp` covering:
   - Railed PID (±90°) × schedule factor 1.0 → crab stays ≤ 85°
   - Railed PID (±90°) × schedule factor 1.2 (Bizzy tune, v=1.5) → crab clamped to ±85°
   - Railed PID (±90°) × schedule factor 1.8 (v=gain_v_min=0.5) → crab clamped to ±85°
   - Assert along-track component ≥ 0: `cos(clamp * pi/180) > 0`
   - Assert linear.x bound: `target_speed / cos(85° * pi/180)` is the worst-case ceiling
   - Low-speed edge (v=0.5, gain_ref_speed=1.8): factor=3.6 → post-clamp = ±85°

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` | Add `kPostScheduleCrabClampDeg = 85.0` constant and clamp `crab_angle` after `gainScheduleScale` call (~line 910) |
| `marine_nav_crabbing_path_follower/test/test_gain_schedule.cpp` | Add `PostScheduleClamp` tests: railed PID × schedule factors {1.0, 1.2, 1.8} + low-speed edge |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Fix restores predictable bounded heading; hardcoded ε prevents an operator from silently re-enabling sail-away via a param. Comment in code explains the invariant. |
| Enforcement over documentation | Unit tests enforce the post-schedule clamp at each of the failure-mode factors. Tests are the load-bearing enforcement. |
| Capture decisions, not just implementations | ε=5° and the anti-windup interaction documented in a code comment; rationale captured here and in progress.md. |
| A change includes its consequences | `turnSpeedFactor` and `cos_crab` both consume the post-schedule angle — both now receive |crab| ≤ 85°, bounding the speed inflation to `target_speed / cos(85°)`. Tests verify this. |
| Only what's needed | Two-file change. Acquisition mode deferred (separate issue). No new configurable parameters. |
| Improve incrementally | Single PR, clear before/after invariant. |
| Test what breaks | Test matrix directly exercises the failure modes from the field bag: railed PID + schedule factors that produced 108°/162° crab. |
| Workspace vs. project separation | All changes in `unh_marine_navigation` project repo. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Already in worktree `issue-unh_marine_navigation-100` on `feature/issue-100`. |
| ADR-0008 — ROS 2 conventions | Yes | Modifying a `nav2_core::Controller` plugin; a clamp-order fix has no convention conflict. |
| ADR-0013 — progress.md vocabulary | Yes | progress.md carries `## Issue Review` + will carry `## Plan Authored`. |
| ADR-0018 — Local-first CI | Yes | Project-repo change; `ci_local.sh` verification required before merge. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `crab_angle` post-schedule clamped to ±85° | `turnSpeedFactor` input now bounded — no change needed, but test coverage improved | Yes — verified by existing turn-speed tests + new clamp tests |
| `crab_angle` post-schedule clamped to ±85° | `cos_crab` floor at 0.5 can only trigger at |crab| ≥ 60° but crab is now ≤ 85°, so worst-case surge = `target_speed / cos(85°) ≈ 11.5×` — this is still a large number but is now finite and predictable | Yes — tests assert `linear.x ≤ target_speed / cos(85° * pi/180)` |
| PID internal clamp stays at ±90° | Anti-windup interaction unchanged — internal integral caps at ±75° (i_min/i_max in configure()); no test change needed for windup | Yes — documented in code comment |
| No new parameters | Parameter descriptors, package docs, README unchanged | Yes (nothing to update) |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
