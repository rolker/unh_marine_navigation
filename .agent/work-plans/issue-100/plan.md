# Plan: CrabbingPathFollower: post-schedule crab angle exceeds ±90°, causing sail-away equilibrium and speed inflation

## Issue

https://github.com/rolker/unh_marine_navigation/issues/100

## Context

In `CrabbingPathFollower::computeVelocityCommands`, the PID is internally clamped at
±90° (`initialize_from_args(…, 90.0, -90.0, …)` — `crabbing_path_follower.cpp:214`),
but `gainScheduleScale` multiplies the output **after** that clamp
(`crabbing_path_follower.cpp:909–911`). At Bizzy's tune (`gain_ref_speed=1.8`,
`target_speed=1.5 m/s`) the factor is 1.2 → |crab| reaches 108°. At the low-speed
floor (`v=gain_v_min=0.5`) the factor reaches 3.6 → |crab| up to 324° pre-wrap.

The harmful consequence when |crab| > 90° is **sail-away**:
`target_heading = base_heading + crab_angle` (`cpp:977`) — the along-track
component goes negative; the heading loop converges to the wrong course and
holds it indefinitely.

**Speed is NOT materially changed by this fix** (plan-review must-fix 2):
`cos_crab = max(cos(crab), 0.5)` (`cpp:1054`) floors the divisor at 0.5, so
`linear.x ≤ 2× target_speed` for any crab angle — before AND after the clamp
(cos(85°) ≈ 0.087 < 0.5, so the floor still governs at the rail; the observed
3.09 m/s ≈ 2 × 1.5 is that floor, not an unbounded blow-up). The clamp is the
sail-away fix; the 2× surge ceiling is pre-existing, intended behavior of the
floor.

**Design decisions (from Issue Review checkpoint 2026-07-23):**
- ε = 5° (hardcoded); post-schedule clamp is ±85°. |crab| < 90° is a correctness
  invariant, not a tuning knob — a configurable limit could be set past perpendicular
  under field pressure and re-introduce sail-away. Matches the existing hardcoded ±90°
  PID init.
- Keep the PID internal clamp at ±90° unchanged. The post-schedule clamp only trims
  the scheduled output; windup stays bounded by the existing conditional-integration
  anti-windup.

## Approach

1. **Add the clamp as a pure inline helper in `path_geometry.hpp`** (plan-review
   must-fix 1 — mirrors `gainScheduleScale`, which is what makes it unit-testable;
   an inline constant in `crabbing_path_follower.cpp` would be unreachable from
   `test_gain_schedule.cpp`, and no `computeVelocityCommands` harness exists):
   ```cpp
   /// ±(90° − ε) post-schedule crab limit; ε = 5°. Correctness invariant, not a
   /// tuning knob (issue #100 — deliberately NOT a ROS parameter).
   constexpr double kPostScheduleCrabClampDeg = 85.0;

   inline double clampPostScheduleCrab(double crab_deg)
   {
     return std::clamp(crab_deg, -kPostScheduleCrabClampDeg, kPostScheduleCrabClampDeg);
   }
   ```

2. **Apply it in `computeVelocityCommands` after gain scheduling** — one line after
   `cpp:911`:
   ```cpp
   crab_angle = AngleDegrees(clampPostScheduleCrab(crab_angle.value()));
   ```
   Add a comment documenting: why ε=5° (correctness invariant, not a knob), the
   anti-windup interaction (internal PID clamp stays at ±90°; the post-schedule
   trim has no windup effect since the integral only accumulates within the PID),
   and the field evidence (bag `2026-07-21T17-26-41`, issue #100).

3. **Add post-schedule clamp tests in `test_gain_schedule.cpp`** — new tests against
   the pure helper (composed with `gainScheduleScale`, exactly how production code
   uses them), schedule **factors** {1.0, 1.2, 3.6} (plan-review suggestion: 1.8 is
   a `gain_ref_speed`, not a factor — at `v = gain_v_min = 0.5` the factor is 3.6):
   - Railed PID (±90°) × factor 1.0 → |post-clamp crab| ≤ 85° (pass-through: 90 → 85)
   - Railed PID (±90°) × factor 1.2 (Bizzy tune, v=1.5 → 108°) → clamped to ±85°
   - Railed PID (±90°) × factor 3.6 (low-speed edge v=gain_v_min=0.5 → 324° pre-wrap) → clamped to ±85°
   - Along-track component positive at the rail: `cos(clampPostScheduleCrab(x) * pi/180) > 0` for railed inputs
   - Surge bound (must-fix 2 reframe): with the clamped angle, `cos_crab = max(cos(crab), 0.5)`
     still floors at 0.5 → assert the effective surge multiplier `1/max(cos(85°·π/180), 0.5) == 2.0`,
     i.e. `linear.x ≤ 2 × target_speed` — unchanged by the fix, now provably finite-and-positive
     along-track
   - Sign symmetry: negative railed inputs clamp to −85°

4. **End-to-end coverage decision** (plan-review suggestion / Issue Review action 4):
   `test_crabbing_control.cpp` has no `computeVelocityCommands` harness today and
   building one is out of scope for this fix. The sail-away invariant is enforced at
   the helper level (pure-function tests above); the composed production call path is
   two adjacent lines (`gainScheduleScale` → `clampPostScheduleCrab`) whose wiring is
   verified by review + the sim zigzag scenario tracked in echoboats#381 thread 2b.
   Action 4 in the Issue Review is resolved as: cover-at-helper-level, end-to-end via
   the #381 sim scenario (not this PR).

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_crabbing_path_follower/include/.../path_geometry.hpp` | Add `kPostScheduleCrabClampDeg = 85.0` + `clampPostScheduleCrab()` inline helper (unit-testable, mirrors `gainScheduleScale`) |
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` | Apply `clampPostScheduleCrab` to `crab_angle` after the `gainScheduleScale` call (~line 911) + invariant comment |
| `marine_nav_crabbing_path_follower/test/test_gain_schedule.cpp` | Add post-schedule clamp tests: railed PID × schedule factors {1.0, 1.2, 3.6}, along-track sign, 2× surge floor, symmetry |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Fix restores predictable bounded heading; hardcoded ε prevents an operator from silently re-enabling sail-away via a param. Comment in code explains the invariant. |
| Enforcement over documentation | Unit tests enforce the post-schedule clamp at each of the failure-mode factors. Tests are the load-bearing enforcement. |
| Capture decisions, not just implementations | ε=5° and the anti-windup interaction documented in a code comment; rationale captured here and in progress.md. |
| A change includes its consequences | `turnSpeedFactor` and `cos_crab` both consume the post-schedule angle — both now receive |crab| ≤ 85°. Surge stays bounded at 2× by the pre-existing 0.5 floor (unchanged); the along-track sign is what the fix repairs. Tests verify both. |
| Only what's needed | Two-file change. Acquisition mode deferred (separate issue). No new configurable parameters. |
| Improve incrementally | Single PR, clear before/after invariant. |
| Test what breaks | Test matrix directly exercises the failure modes from the field bag: railed PID + schedule factors that produced 108°/324° crab. |
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
| `crab_angle` post-schedule clamped to ±85° | `cos_crab = max(cos(crab), 0.5)` still floors at 0.5 (cos(85°) ≈ 0.087 < 0.5), so worst-case surge stays `2 × target_speed` — unchanged by this fix; what changes is the along-track component sign (always positive post-clamp), eliminating the sail-away equilibrium | Yes — tests assert the 2× floor bound and positive along-track (plan-review must-fix 2) |
| PID internal clamp stays at ±90° | Anti-windup interaction unchanged — internal integral caps at ±75° (i_min/i_max in configure()); no test change needed for windup | Yes — documented in code comment |
| No new parameters | Parameter descriptors, package docs, README unchanged | Yes (nothing to update) |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
