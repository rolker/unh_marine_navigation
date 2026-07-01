# Plan: Anticipatory speed regulation via path curvature

## Issue

https://github.com/rolker/unh_marine_navigation/issues/89

## Context

Issue #87 (PR #88, merged) added **reactive** turn-speed regulation: the boat
slows in proportion to the crab angle it is holding. This is inherently a beat
late — by the time the PID commands a large crab angle, the boat is already
committed into the turn at speed.

This issue adds the **anticipatory** half: a curvature-based speed cap that slows
the boat *before* the turn apex by looking ahead on the path geometry. The two
compose via `min()` so both regimes apply simultaneously.

The implementation lives entirely in `marine_nav_crabbing_path_follower` and
follows the same pattern as `turnSpeedFactor` (#87):

- **Algorithm**: 3-point circumscribed-circle fit using the boat's current plan
  position, a half-lookahead point, and the full-lookahead point (reuses the
  existing `lookaheadPoint()` machinery). Chosen over heading-change-per-arc-length
  because the circumfit degeneracy case (collinear → R = ∞ → factor 1.0) is direct
  and requires no arc-length measure on a piecewise-linear path.
- **Param**: `turn_speed_curvature_min_radius` (not `turn_curvature_min_radius`),
  units m, default 0.0 (disabled), group `speed` — naming follows the existing
  `turn_speed_*` namespace (review-issue recommendation).
- **Floor**: reuses `turn_speed_min_factor_` (no new floor param) — since the
  two regulators already share a `min()` composition, a single floor is simpler
  and avoids an operator tuning two separate minimums for what is effectively
  one "slowest I'm willing to go in a turn" knob.
- **Disabled** when `turn_speed_curvature_min_radius = 0.0` OR when lookahead
  is disabled (`lookahead_distance = 0`, `lookahead_time = 0`) — factor returns
  1.0 in both cases, a clean no-op that changes no existing behavior.

## Approach

1. **Add `circumscribedRadius()` to `path_geometry.hpp`** — pure inline function
   taking 3 `geometry_msgs::msg::Point` args. Returns `std::numeric_limits<double>::infinity()`
   for degenerate inputs (collinear, zero-area, fewer than 3 distinct points, NaN
   coordinates). Formula: R = |AB|·|BC|·|CA| / (4·|area|) where area is the
   signed area of the triangle.

2. **Add `curvatureSpeedFactor()` to `path_geometry.hpp`** — pure inline function
   `(double radius, double min_radius, double min_factor) → double`. Contract:
   - `min_radius <= 0` → returns 1.0 (disabled).
   - `radius` non-finite or ≥ `min_radius` → returns 1.0 (straight/gentle).
   - Otherwise: `clamp(radius / min_radius, min_factor, 1.0)`.
   A tight turn (R < R_min) yields a factor < 1 that slows the boat; R ≥ R_min
   (a gentle bend) yields 1.0, leaving speed untouched.

3. **Add `turn_speed_curvature_min_radius_` atomic to `crabbing_path_follower.h`**
   — `std::atomic<double>`, default 0.0, with the same doc-comment style as
   `turn_speed_max_crab_deg_`.

4. **Add one `kTunables[]` entry in `crabbing_path_follower.cpp`** —
   `{"turn_speed_curvature_min_radius", 0.0, 0.0, 200.0, "m", "speed", "..."}`.
   Picked up automatically by `declareCrabbingControlParams` / `bindCrabbingControls`,
   growing the control count from 12 to 13.

5. **Add `read_validated` call in `configure()`** — same pattern as
   `turn_speed_max_crab_deg_`, `lo=0.0, exclusive_lo=false`.

6. **Add callback branch in the on-set-parameters callback** — `>= 0.0` lower
   bound (0.0 = disabled is valid), no upper bound (wide open, operator-settable).

7. **Apply curvature factor in `computeVelocityCommands()`**:
   - Hoist `la_point` out of the existing `if (lookahead > 0.0)` block so it is
     in scope at the regulation site.
   - At the turn-speed regulation site (immediately before the `cos_crab`
     division, after the trajectory-speed rederivation block), compute:
     ```
     curvature_factor = 1.0
     if lookahead > 0 and curvature_min_radius > 0:
       half_la = lookaheadPoint(..., lookahead / 2)
       R = circumscribedRadius(pose_in_plan.pose.position, half_la, la_point)
       curvature_factor = curvatureSpeedFactor(R, curvature_min_radius, turn_min_factor)
     combined_factor = min(turn_factor, curvature_factor)
     regulated_target_speed = target_speed * combined_factor
     ```
   - Snapshot `turn_speed_curvature_min_radius_` atomic once (same tear-free
     pattern as lookahead_* and gain-schedule atomics).
   - Extend the existing DEBUG log to also print `curvature_factor` and `combined_factor`.

8. **Add `test/test_curvature_speed_factor.cpp`** — unit tests using
   `circumscribedRadius` and `curvatureSpeedFactor` directly:
   - Straight path (collinear 3 pts) → R = ∞ → factor 1.0.
   - Constant-radius arc (equilateral triangle inscribed in a known circle) →
     expected factor.
   - Degenerate: NaN coordinate in a point → R = ∞ → factor 1.0.
   - Degenerate: `min_radius = 0` → factor 1.0.
   - `R >= min_radius` (gentle bend) → factor 1.0.
   - `R < min_radius` (tight turn) → factor R/min_radius, floored at min_factor.
   - Floor respected: tiny R → min_factor not underrun.
   - Symmetric: no dependence on point ordering that would break (regression guard).

9. **Update `CMakeLists.txt`** — add `ament_add_gtest(test_curvature_speed_factor ...)` target wired like the sibling `test_turn_speed_factor` target.

10. **Update `test/test_crabbing_control.cpp`** — bump size assertions 12u → 13u
    (both the advertise test and the wrapped-namespace test), update the "twelve
    controls" doc comment, and add group/units/range assertions for the new
    curvature control (`speed` group, `m` units, range `[0, 200]`).

## Files to Change

| File | Change |
|------|--------|
| `include/marine_nav_crabbing_path_follower/path_geometry.hpp` | Add `circumscribedRadius()` and `curvatureSpeedFactor()` |
| `include/marine_nav_crabbing_path_follower/crabbing_path_follower.h` | Add `turn_speed_curvature_min_radius_` atomic member + doc comment |
| `src/crabbing_path_follower.cpp` | Add `kTunables[]` entry; `read_validated`; callback branch; apply curvature factor in `computeVelocityCommands` |
| `test/test_curvature_speed_factor.cpp` | New unit test file (8 test cases) |
| `CMakeLists.txt` | Add `test_curvature_speed_factor` gtest target |
| `test/test_crabbing_control.cpp` | 12u → 13u in two size assertions; update doc comment; add curvature control assertions |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Default `turn_speed_curvature_min_radius = 0.0` is a clean no-op; live-settable via marine_control; all three opt-in conditions (default-off, live-tunable, finiteness-guarded) satisfied |
| A change includes its consequences | All six files in scope; `declareCrabbingControlParams`/`bindCrabbingControls` pick up the new entry automatically via the `kTunables` loop; `test_crabbing_control.cpp` size + group/units/range assertions updated |
| Only what's needed | Reuses `lookaheadPoint()` (no second horizon); reuses `turn_speed_min_factor_` (no second floor param); degenerate guard in pure function only |
| Improve incrementally | Single PR; default-off; builds directly on the #87 pattern |
| Test what breaks | 8 curvature unit tests + `test_crabbing_control` count/group checks cover the acceptance criteria |
| Safety First | Degenerate inputs (collinear, <3 pts, NaN, R=0, min_radius=0) all return factor 1.0 (no slowdown is the safe side for degenerate geometry); combined_factor ≤ 1.0 so the boat can only slow, never speed up |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Already in worktree `issue-unh_marine_navigation-89` ✓ |
| 0008 — ROS 2 conventions | Yes | `kTunables` entry, `FloatingPointRange` descriptor, `_range` companion, `read_validated`, callback branch all follow the established pattern exactly |
| 0013 — progress.md vocabulary | Yes | This plan and the `## Plan Authored` entry follow ADR-0013 ✓ |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `kTunables[]` (new entry) | `test_crabbing_control.cpp` size assertions (12u → 13u) + group/units assertions | Yes |
| `path_geometry.hpp` (new functions) | `test_curvature_speed_factor.cpp` unit tests; CMakeLists.txt | Yes |
| Control count doc comments ("twelve controls") | Update to "thirteen" in `test_crabbing_control.cpp` header | Yes |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
