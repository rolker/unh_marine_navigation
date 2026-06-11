# Plan: Cross-track controller over-corrects into 360° loops on planner paths (yaw-ceiling × path-jump synergy)

## Issue

https://github.com/rolker/unh_marine_navigation/issues/66

## Context

#72/#70 merged (progress-preserving localization in `setPlan`, bounded one-segment
backward re-localization, conditional-integration anti-windup, pure-pursuit look-ahead)
and **stopped the full 360° loops**. But the 2026-06-10 deployment (#250) run-bag RCA shows
**residual hunting that is gated by planner REPLAN CHURN, not speed**: a *stable* reference
tracks sub-metre even at ~3 kn; mid-line re-sends / `override goto` / `replace_task` trigger
replan bursts that step the cross-track reference and saturate the (currently pure-proportional:
Kp=1.0, Ki=Kd=0) controller. Two churn sources remain after #72:

1. **`AvoidanceController`** reshapes the path and re-issues `primary_->setPlan(reshaped)`
   **every control cycle** (`computeVelocityCommands`), so the near-boat geometry varies each
   tick even on clear water (`kDeviationEpsilon=0.05` is only used for viz today, not to
   suppress churn).
2. **The BT / global planner re-emits `/bizzy/plan`** on replans (`run_tasks.xml`), each new
   short/curved path stepping the cross-track reference.

Fix both fronts (per the #250 analysis: stabilize the reference at source **and** make the
controller robust to residual steps).

## Approach

**Front A — stabilize the reference at the source**

1. **AvoidanceController reshape suppression + hysteresis** — in `computeVelocityCommands`,
   when peak `|offset|` < `kDeviationEpsilon` (no meaningful obstacle deviation), pass the
   **nominal** plan to the primary controller and only re-issue `primary_->setPlan` when the
   reshaped geometry actually changes beyond a threshold — instead of a freshly-resampled
   near-nominal path every cycle. Removes clear-water churn without touching real avoidance.
2. **BT replan hysteresis** (`run_tasks.xml`) — don't re-emit a new global plan while the
   current path is still valid (`IsPathValid`) and the goal is unchanged; rate-limit replans so
   a still-good survey line is not re-planned out from under the follower.

**Front B — make the controller robust to residual reference steps**

3. **CrabbingPathFollower slew limit** — in `computeVelocityCommands`, slew-limit the
   cross-track **error** fed to the PID (parameterized, finite/domain-validated like the
   existing `default_speed` handling) so a residual reference step can't slam the yaw output.
   Decoupled from and upstream of the velocity_smoother hard clamp (the safety floor stays).
   Optionally add a small derivative term using **derivative-on-measurement** (not on error,
   to avoid derivative kick on reference steps), param-gated, default 0 pending field tuning.
4. **Regression tests** (covers nav#5 scope) — a controller unit test feeding a stepping
   reference and asserting bounded yaw / no limit cycle; a stable-reference-at-speed test
   asserting sub-metre cross-track at ≥1.5 m/s.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_avoidance_controller/src/avoidance_controller.cpp` | Reshape suppression + hysteresis (A.1) |
| `marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml` | Replan hysteresis / rate-limit (A.2) |
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` (+ `.h`) | Cross-track-error slew limit + optional derivative-on-measurement (B.3) |
| `marine_nav_crabbing_path_follower/test/*` (+ avoidance test) | Regression tests (B.4) |
| controller config YAML (params) | New tunables: slew limit, hysteresis thresholds, derivative gain (default 0) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Quality Standard (robustness + tests) | Fix the over-correction completely on both fronts; ship regression tests, not a partial damp |
| Validation / no silent failure | New params finite + domain-checked, mirroring the controller's existing `default_speed` validation + param-callback pattern |
| Safety (robot on open water) | Changes sit upstream of the velocity_smoother clamp and Collision Monitor reflex — the safety floor is unchanged; reshape suppression must not suppress *real* obstacle avoidance |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (follow ROS 2 conventions) | Yes | nav2 controller/BT plugin patterns; params via `declare_parameter` with descriptors + validation callbacks |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| Avoidance reshape gating | Verify it still deviates when an obstacle IS present (test clear + obstacle) | Yes (B.4 tests) |
| BT replan hysteresis | Verify `IsPathValid` still forces a replan on a genuinely blocked path (no avoidance stall) | Yes |
| New controller params | controller config YAML + package docs | Yes |
| Controller output dynamics | Re-verify interaction with velocity_smoother clamp + CA gate in sim | Yes (sim gate) |

## Open Questions

- Slew-limit the cross-track **error** (preserves PID semantics) vs the **output** yaw (simpler)? Recommend error-side.
- Enable the derivative-on-measurement term this PR, or land it disabled (default 0) pending field tuning? Recommend disabled-by-default.
- One PR (A+B+tests) or split A (reference stabilization) → B (controller robustness)? Recommend one cohesive PR; split if review prefers smaller diffs.

## Estimated Scope

Single PR (A+B+tests), medium. Splittable into A then B if review prefers. Sim-verify a jumpy-reference scenario before/after; field-confirm on the next deployment.
