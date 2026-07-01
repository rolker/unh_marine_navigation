# Plan: Add turn-speed regulation to CrabbingPathFollower

## Issue

https://github.com/rolker/unh_marine_navigation/issues/87

## Context

`CrabbingPathFollower.computeVelocityCommands()` (line 879–880) applies
`target_speed / cos(crab_angle)` as the commanded surge. When the cross-track PID
drives a large crab angle on a turn, this **inflates** `linear.x` right when the
boat is turning hardest. Field data (2026-06-30, quad coulomb model) shows +18%
commanded surge and +60% modelled current draw in turns vs. straights.

The fix inserts a `turnSpeedFactor()` regulation step: a multiplicative factor in
`[min_factor, 1.0]` applied to `target_speed` **before** the `cos_crab` division.
The factor is driven by the magnitude of the already-computed crab angle (after
gain scheduling). Default off → backward-compatible. Crab-angle is chosen over
path-curvature because it is already in-loop; curvature-based anticipation is a
natural follow-on.

**Signal choice rationale** (closes the review-issue open action): path curvature
requires a 3-point lookahead walk, degenerate-input handling, and a separate
`R_min` param. The crab angle already encodes how hard the controller is
correcting — a large `|crab_angle|` means the boat is off-track or turning — and
arrives with no extra geometry. The simpler path is taken first; curvature can
add anticipation in a follow-on issue.

## Approach

1. **Add `turnSpeedFactor()` to `path_geometry.hpp`** — pure inline function;
   disabled when `max_crab_deg <= 0` (returns 1.0); otherwise computes
   `clamp(1 - |crab_angle_deg| / max_crab_deg, min_factor, 1.0)`. Follows the
   `gainScheduleScale` pattern: pure, no ROS, unit-testable without scaffolding.
   Guard NaN/Inf crab (treat as max_crab_deg magnitude → min_factor).

2. **Add two atomics to the class header** — `turn_speed_max_crab_deg_`
   (default `0.0`, disables regulation) and `turn_speed_min_factor_`
   (default `0.3`, floors the speed at 30% of `target_speed` when enabled).

3. **Add two entries to `kTunables[]`** in `.cpp`:
   - `{"turn_speed_max_crab_deg", 0.0, 0.0, 90.0, "deg", "speed", "..."}`
   - `{"turn_speed_min_factor", 0.3, 0.0, 1.0, "", "speed", "..."}`
   `declareCrabbingControlParams` and `bindCrabbingControls` iterate `kTunables`
   and pick them up automatically — no extra binding code.

4. **Add `read_validated` calls in `configure()`** — same pattern as existing
   nine params, both with `lo = 0.0, exclusive_lo = false` (`>= 0`):
   ```cpp
   read_validated(".turn_speed_max_crab_deg", turn_speed_max_crab_deg_, 0.0, false);
   read_validated(".turn_speed_min_factor",   turn_speed_min_factor_,   0.0, false);
   ```

5. **Add callback branches in the `on_set_parameters_callback`** —
   `max_crab_deg`: `update(..., 0.0, false)` (`>= 0`; 0 disables);
   `min_factor`: `update(..., 0.0, false)` plus an explicit upper-bound check
   (reject if `v > 1.0`) using the same `result.successful = false` + early-
   return pattern the existing callback uses.

6. **Apply `turnSpeedFactor` in `computeVelocityCommands`** — after
   `gainScheduleScale` (which produces the final `crab_angle`), before the
   `cos_crab` division (line 879). Snapshot both new atomics once per cycle:
   ```cpp
   const double turn_max_crab = turn_speed_max_crab_deg_.load();
   const double turn_min_factor = turn_speed_min_factor_.load();
   target_speed *= turnSpeedFactor(crab_angle.value(), turn_max_crab, turn_min_factor);
   // existing:
   double cos_crab = std::max(cos(crab_angle), 0.5);
   cmd_vel.twist.linear.x = target_speed / cos_crab;
   ```
   Add a DEBUG log line after regulation so field engineers can see the regulated
   speed alongside the prior `target_speed` log.

7. **Add unit tests in `test/test_turn_speed_factor.cpp`** (new file, same
   pattern as `test_gain_schedule.cpp`):
   - `DisabledReturnsOne` — `max_crab_deg <= 0` → 1.0 always
   - `ZeroCrabReturnsOne` — `crab_angle = 0` → 1.0 (no slowdown on straights)
   - `CrabAtMaxReturnsMinFactor` — `|crab| >= max_crab_deg` → `min_factor`
   - `LinearRamp` — intermediate crab → expected linear value
   - `SymmetricForNegativeCrab` — `|crab|` (negative crab same slowdown)
   - `NonFiniteCrabClampsToMinFactor` — NaN/Inf → `min_factor` (no blow-up)
   - `MinFactorFloor` — factor never goes below `min_factor`

8. **Wire test in `CMakeLists.txt`** — add `ament_add_gtest(test_turn_speed_factor ...)`
   with the same include/dep pattern as `test_gain_schedule`.

## Files to Change

| File | Change |
|------|--------|
| `include/marine_nav_crabbing_path_follower/path_geometry.hpp` | Add `turnSpeedFactor()` inline function |
| `include/marine_nav_crabbing_path_follower/crabbing_path_follower.h` | Add `turn_speed_max_crab_deg_`, `turn_speed_min_factor_` atomics with doc comments |
| `src/crabbing_path_follower.cpp` | Add two `kTunables` entries; two `read_validated` calls; two callback branches (with upper-bound check for `min_factor`); `turnSpeedFactor` call in `computeVelocityCommands`; DEBUG log for regulated speed |
| `test/test_turn_speed_factor.cpp` | New unit-test file for `turnSpeedFactor` (7 test cases) |
| `CMakeLists.txt` | Add `ament_add_gtest(test_turn_speed_factor ...)` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Default off (`max_crab_deg = 0`); live-settable via `marine_control` panel and `ros2 param set`; all new params have descriptors, FloatingPointRange, and finiteness guards |
| A change includes its consequences | Unit tests, param declarations, marine_control binding, and DEBUG log all in the same PR; no interface changes (same `TwistStamped` output) |
| Only what's needed | Two params, one pure function, one test file; crab-angle reuses already-computed in-loop state; no new geometry walk |
| Improve incrementally | Single PR; backward-compatible default; does not preclude curvature-based follow-on |
| Test what breaks | Unit tests on `turnSpeedFactor` cover: disabled, zero-crab no-op, max-crab floor, linear ramp, symmetry, NaN/Inf defense |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Already in `feature/issue-87` worktree ✓ |
| 0008 — ROS 2 conventions | Yes | `nav2_util::declare_parameter_if_not_declared`; `FloatingPointRange` descriptor; `add_on_set_parameters_callback` pattern — identical to existing params |
| 0001 — Adopt ADRs | Recommendation | Signal-choice rationale (crab-angle over curvature) recorded in §Context; no separate ADR needed unless approach changes during implementation |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| New params in `kTunables[]` | `declareCrabbingControlParams` / `bindCrabbingControls` pick them up automatically; no marine_control-side changes needed | Yes |
| Speed precedence (#32) | Regulation is a `*=` on `target_speed` after the precedence winner (default_speed → speed_limit_ cap → trajectory speed override) and before the `cos_crab` division — same slot as `speed_limit_` cap; no new speed source | Yes |
| DEBUG log for regulated speed | Add one `RCLCPP_DEBUG_STREAM` line showing the regulated `target_speed` before the `cos_crab` line | Yes (step 6) |
| `min_factor` upper-bound validation | The generic `update()` lambda only checks a lower bound; add an explicit `v > 1.0` rejection in the `min_factor` callback branch | Yes (step 5) |

## Open Questions

- [ ] Sim acceptance criterion: no specific gazebo launch / scenario is pinned in the package. The regulation behavior can be validated via topic logging (`/autonomous/cmd_vel`, `/odom`) on a zig-zag path; if a dedicated sim scenario is required before merge, it must be added to the plan.

## Estimated Scope

Single PR.
