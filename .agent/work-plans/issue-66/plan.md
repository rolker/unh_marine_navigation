# Plan: Cross-track controller over-corrects into 360° loops on planner paths (yaw-ceiling × path-jump synergy)

## Issue

https://github.com/rolker/unh_marine_navigation/issues/66

## Context

#72/#70 merged (progress-preserving localization in `setPlan`, bounded one-segment
backward re-localization, conditional-integration anti-windup, pure-pursuit look-ahead)
and **stopped the full 360° loops**. The 2026-06-10 deployment (#250) run-bag RCA showed
**residual hunting gated by planner REPLAN CHURN, not speed**: a *stable* reference tracks
sub-metre even at ~3 kn; mid-line re-sends / `override goto` / `replace_task` trigger replan
bursts that step the cross-track reference and saturate the (pure-proportional: Kp=1.0,
Ki=Kd=0) controller. Two churn sources remain: the `AvoidanceController` reshapes + re-issues
`setPlan` every cycle, and the global planner re-emits `/bizzy/plan` on replans.

This is delivered in **first-cut + follow-ups** so there's a testable artifact fast: the
first cut makes the controller robust to *any* reference step regardless of source; the
follow-ups reduce the churn at the source.

## Approach

### First cut (this PR) — controller robustness, B.3 only

**Cross-track-error slew limiter** in `CrabbingPathFollower::computeVelocityCommands`: a pure
`slewToward()` helper (in `path_geometry.hpp`) caps how fast the cross-track error *fed to
the PID* may change (`cross_track_error_slew_rate_`, m/s). A discontinuous reference step is
ramped in instead of slamming the controller; genuine lateral drift (far slower than a sane
rate) passes through untouched. Re-seeded on the same fresh-start gate as the PID reset so a
post-gap resume snaps to the current error rather than crawling from a stale value. **Default
0.0 = disabled (historical behaviour)**; the raw error is still logged as the true tracking
error. Live-tunable via the existing parameter callback (mirrors the look-ahead/heading param
plumbing + validation). Unit-tested via `slewToward` (`test_path_geometry.cpp`): ramps a
step, snaps within a step, never overshoots, disables on rate ≤ 0.

### Follow-ups (separate PRs/issues — NOT in this PR)

- **A.1 — AvoidanceController reshape suppression + hysteresis**: pass the nominal path
  through when peak `|offset|` < `kDeviationEpsilon`, only re-issue `primary_->setPlan` when
  the reshaped geometry materially changes. Removes clear-water churn at the source.
- **A.2 — BT replan hysteresis** (`run_tasks.xml`): don't re-emit a global plan while the
  current path is still valid (`IsPathValid`) and the goal is unchanged (gate
  `ComputePathToPose` / tune `RateController`). Specific BT mechanism TBD during A.2.
- **B.3b — derivative-on-measurement term**: add a small damping term computed on the boat's
  cross-track *position* (not the error, to avoid derivative kick on reference steps),
  param-gated, default 0 — land disabled, tune in the field.
- **#66 fix-direction (2) — survey yaw-rate magnitude cap**, decoupled from the smoother's
  1.0 rad/s capability ceiling. **Deferred:** the slew-limit removes the step *excitation* the
  data fingered, and #250 showed a stable reference tracks clean at 3 kn — so a magnitude cap
  is likely unnecessary; revisit only if hunting survives the first cut + A.1/A.2.

## Files to Change (this PR)

| File | Change |
|------|--------|
| `marine_nav_crabbing_path_follower/include/.../path_geometry.hpp` | `slewToward()` pure helper |
| `marine_nav_crabbing_path_follower/include/.../crabbing_path_follower.h` | Slew-rate atomic param + slew state members |
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` | Declare/validate/live-tune the param; slew the cross-track error before the PID; re-seed on reset |
| `marine_nav_crabbing_path_follower/test/test_path_geometry.cpp` | `SlewToward` unit tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Quality Standard (robustness + tests) | First cut ships with unit tests; default-off so it can't regress unvalidated behaviour |
| Validation / no silent failure | Param finite + `>= 0` checked, reusing the controller's `read_validated` / callback pattern |
| Safety (robot on open water) | Slew sits upstream of the velocity_smoother clamp + Collision Monitor reflex — the safety floor is unchanged; default-off means no behaviour change until a deployment opts in |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (follow ROS 2 conventions) | Yes | `declare_parameter_if_not_declared` + validating param callback; nav2 controller plugin idiom |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| Add `cross_track_error_slew_rate` param | **Deployed value lives in `unh_echoboats_project11` (`bizzyboat_project11/config/nav2_overlay.yaml`), not this repo** — set it there (e.g. ~3.0) to enable on the boat; sim can `ros2 param set` live | No — cross-repo follow-up, flagged |
| Controller output dynamics | Re-verify interaction with velocity_smoother clamp + CA gate in sim | Yes (sim gate) |
| Slew the error | Confirm genuine cross-track recovery still tracks (rate ≥ ~2× lateral drift) — pick the deployed rate in sim | Yes |

## Open Questions (resolved)

- Slew the **error** (chosen — preserves PID semantics) vs the output yaw.
- Derivative term: **land disabled / defer to B.3b** (chosen).
- **Slew-limit first**, A.1/A.2/yaw-cap as follow-ups (chosen, over one big PR).

## Estimated Scope

First cut: single small PR (one package, ~80 lines + tests). Follow-ups A.1, A.2, B.3b, and
the yaw-rate cap are separate. Sim-verify a jumpy-reference scenario with the rate set; field
test on the next deployment.

## Implementation Notes

- **Descoped to slew-limit-first** (vs the original one-PR-both-fronts): the #250 data says
  reference-step excitation is the trigger and the slew-limit neutralizes it regardless of
  which churn source caused the step, so it's the fastest testable artifact and the
  highest-leverage single change. Source-side churn reduction (A.1/A.2) becomes additive
  follow-up rather than a prerequisite.
- **Default 0.0 (disabled)** matches the package convention (look-ahead/heading params all
  default to historical no-op) and avoids changing a shared controller's behaviour for sim +
  other platforms without per-platform validation; the deployed config opts in.
- **Re-acquisition is throttled too, by design** (review-code adversarial finding). The
  limiter caps the error *magnitude* rate, so a genuinely large offset — mission start far
  from the line, a big avoidance excursion, or re-acquisition after a gap shorter than
  `pid_reset_threshold_` — is also ramped in at `rate` m/s, not just replan steps. Choose the
  deployed rate with re-acquisition in mind (not only anti-hunt); a gap longer than the reset
  threshold re-seeds and snaps. The slew state lives in the testable `slewLimitError` helper.
- **`dt <= 0` holds, not passes through** (review-code adversarial finding): a zero /
  duplicate-stamp control cycle must not let `slewToward`'s "non-positive step = passthrough"
  leak the raw jump and defeat the limiter; `slewLimitError` holds the previous value there.
