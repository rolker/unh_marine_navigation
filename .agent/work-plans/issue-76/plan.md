# Plan: marine_nav_crabbing_path_follower — speed-normalize the cross-track gain (gain schedule)

## Issue

https://github.com/rolker/unh_marine_navigation/issues/76

## Context

`CrabbingPathFollower` (`marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp`)
is a cascade controller: cross-track error → slew-limit (#66) → PID → `crab_angle`
(heading offset). The outer (cross-track) loop has `ė ≈ v·sin(crab_angle) ≈ v·(p·e)`,
so the outer-loop gain is **proportional to commanded speed `v`**. With fixed PID
gains the stability margin shrinks as speed rises, producing a speed-dependent
cross-track limit cycle (±1 m, ~10 s, non-decaying at 3.5 kt; stable at 3 kt;
observed 2026-06-16 Lake Massabesic, rolker/unh_echoboats_project11#289). A flat
field mitigation (`pid.p −20 → −13`) works but doesn't hold the response constant
across speed.

The proper fix is to **speed-normalize the PID output** by `gain_ref_speed / v`
right after the PID compute (L528), cancelling the broadband plant gain `v`. This
is opt-in: default `gain_ref_speed = 0.0` disables it, so there is **no behavior
change** until a platform configures it — exactly mirroring the `lookahead_time = 0`
default-off pattern already in this file.

The package already follows a clean separation: pure control/geometry math lives in
the header `path_geometry.hpp` and is unit-tested in `test/test_path_geometry.cpp`
without spinning up a ROS node. The two live-tunable atomics follow the
`lookahead_*` pattern: `std::atomic<double>` member + `read_validated()` at
configure + a `SetParameters` branch (with the `as_number` integer-coercion guard).

## Approach

> **Implementation status (feature/issue-76):** all 8 steps below are DONE.
> Param keys are sub-namespaced under `.pid.` per the Plan Review finding
> (`progress.md` Plan Review): `plugin_name_ + ".pid.gain_ref_speed"` /
> `".pid.gain_v_min"` in the declare (`read_validated` suffix), read, and
> SetParameters branches — mirroring `".pid.reset_threshold_seconds"`, not the
> bare-suffix `lookahead_*` keys. No other deviations.

1. **Extract the scaling as a pure function** — [DONE] add
   `gainScheduleScale(crab_angle_deg, gain_ref_speed, v_min, target_speed)` to
   `path_geometry.hpp` (the established home for pure, unit-testable control math).
   Contract: `gain_ref_speed <= 0` returns `crab_angle_deg` unchanged (disabled,
   default); otherwise returns `crab_angle_deg * gain_ref_speed / max(target_speed, v_min)`.
   `v_min` floors the effective speed so creep/station-keep (`target_speed → 0`)
   can't blow the gain up. Keeping it pure (not a method) is what lets the unit
   test exercise it across speeds with no ROS scaffolding — the same reason
   `slewLimitError` / `lookaheadPoint` live there.

2. **Add the two atomic members** to `crabbing_path_follower.h` following the
   `lookahead_*` block: `std::atomic<double> pid_gain_ref_speed_{0.0};` and
   `std::atomic<double> pid_gain_v_min_{0.5};`, with a block comment explaining
   the `v`-cancellation rationale, the default-off semantics, and the floor.

3. **Declare + validate at configure** — add two `read_validated(...)` calls.
   `gain_ref_speed` uses `lo=0.0, exclusive_lo=false` (`>= 0`, so 0 = disabled is
   valid). `gain_v_min` uses `lo=0.0, exclusive_lo=true` (`> 0`, a floor of 0
   would re-admit the divide-by-near-zero blow-up).

4. **Add the two SetParameters branches** — mirror the `lookahead_*` branches
   exactly: `as_number(...)` coercion then `update(target, v, lo, exclusive_lo, name)`,
   same `>= 0` / `> 0` split as step 3, so both params are live-tunable.

5. **Call the scaler in `computeVelocityCommands`** right after L528
   (`crab_angle = AngleDegrees(pid_->compute_command(...))`), snapshotting both
   atomics once (intra-cycle tear-free, matching the `lookahead_*` snapshot idiom):
   ```cpp
   crab_angle = AngleDegrees(gainScheduleScale(
     crab_angle.value(), pid_gain_ref_speed_.load(), pid_gain_v_min_.load(),
     target_speed));
   ```
   Placed **before** the L582 trajectory-speed reassignment, so it uses the
   commanded `target_speed` (L410), not the per-pose trajectory speed.

6. **In-source rationale comment** at the insertion point: why **commanded**
   `target_speed` (keeps the gain term out of the measured-speed feedback path —
   a feedback-coupled gain would itself be a dynamic element); why the `v_min`
   floor (no blow-up at creep / station-keep); and the `gain_ref_speed / v`
   = plant-gain-cancellation reasoning.

7. **Unit test** — new `test/test_gain_schedule.cpp` (gtest, pure, no ROS), added
   to `CMakeLists.txt` with a second `ament_add_gtest(...)` block mirroring
   `test_path_geometry`. Cases:
   - **disabled**: `gain_ref_speed = 0` returns the input crab angle unchanged
     (and a negative `gain_ref_speed` too).
   - **enabled, multiple speeds**: at `v = gain_ref_speed` the angle is unchanged
     (unity); at `v < ref` it scales up; at `v > ref` it scales down; check the
     exact `ref/v` ratio at 2–3 speeds.
   - **gain_v_min floor**: with `target_speed` below `v_min`, the divisor is
     `v_min` (not `target_speed`), so the result is bounded — assert it equals
     the `v_min`-divided value and that `target_speed = 0` does not produce
     inf/nan.
   - **sign preservation**: a negative crab angle stays negative after scaling.

8. **Parameter documentation** — the package has **no** README or param-reference
   doc, and the repo-root `README.md` documents no controller params (the
   existing `lookahead_*` / `cross_track_error_slew_rate` params are documented
   only in-source). So the in-source block comments (steps 2 + 6) are the
   parameter documentation; there is no separate doc to update. (Noted as a
   consequence below; a package-doc task is out of scope for this fix.)

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_crabbing_path_follower/include/marine_nav_crabbing_path_follower/path_geometry.hpp` | Add pure `gainScheduleScale()` with doc comment |
| `marine_nav_crabbing_path_follower/include/marine_nav_crabbing_path_follower/crabbing_path_follower.h` | Add `pid_gain_ref_speed_` / `pid_gain_v_min_` atomics + rationale comment |
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` | `read_validated` ×2, SetParameters branch ×2, scaler call + rationale comment at L528 insertion point |
| `marine_nav_crabbing_path_follower/test/test_gain_schedule.cpp` | New gtest: disabled, multi-speed scaling, `v_min` floor, sign |
| `marine_nav_crabbing_path_follower/CMakeLists.txt` | Add `ament_add_gtest(test_gain_schedule ...)` block |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Robustness / completeness (Quality Standard) | Default-off (`gain_ref_speed = 0`) means zero behavior change until a platform opts in; `v_min` floor + validation (`> 0`) prevents divide-by-zero / NaN cmd_vel on an autonomous boat — handled, with a test asserting `target_speed = 0` stays finite. |
| Fix it completely (add the test, handle the edge case) | The plan ships the unit test (multi-speed, disabled, floor, sign) and the creep/station-keep edge case in the same PR, not "good enough". |
| Verify before documenting | In-source param docs only — verified there is no README/param-ref doc to drift; the issue's analysis (L-numbers, scope) was checked against the actual source before planning. |
| Don't invent conventions; follow precedent | Implementation models the existing `lookahead_*` live-tunable pattern exactly (atomic + read_validated + SetParameters + snapshot), and the pure-math-in-header / gtest pattern of `path_geometry.hpp`. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (Follow ROS 2 conventions) | Yes | Nav2 controller-plugin idiom preserved: params declared via `nav2_util::declare_parameter_if_not_declared`, live updates via the existing on-set-parameters callback, plugin-name-prefixed param names. No new conventions invented. |
| ADR-0013 (`progress.md` vocabulary) | Yes | This phase appends a `## Plan Authored` entry; implementation/review phases append their own entry types. |
| Others (0001–0012, 0014) | No | Workspace-infra / mode ADRs; not implicated by a controller code change. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `pid.gain_ref_speed` / `pid.gain_v_min` params | Platform nav2 overlay (`unh_echoboats_project11` `nav2_overlay.yaml`) to activate `gain_ref_speed: 1.8`, `gain_v_min: 0.5` | **No — deferred.** Out of scope per operator decision; the 1.8 anchor needs a sim + on-water re-test first (issue Caveat). A follow-on issue should be opened in `unh_echoboats_project11` to activate after re-test. |
| Add new live-tunable params | Parameter reference doc | N/A — no README/param-ref doc exists for this package; in-source comments are the doc. A dedicated package-documentation task (separate issue) would be the place to add a README. |
| Add a new test source | `CMakeLists.txt` `BUILD_TESTING` block | Yes — `ament_add_gtest(test_gain_schedule ...)`. |

## Open Questions

- None blocking. The `gain_v_min` default (~0.5 m/s) and the `gain_ref_speed`
  anchor (1.8 m/s) come from the issue; only `gain_v_min`'s default ships in
  this PR (`gain_ref_speed` ships as 0 = disabled), and the 1.8 activation is
  the deferred follow-on. If the reviewer prefers `gain_v_min` as `>= 0`
  (disabled-floor semantics) rather than `> 0`, flag at review — the plan
  chooses `> 0` to guarantee the divisor can never reach zero.

## Estimated Scope

Single PR in `unh_marine_navigation` only. The `unh_echoboats_project11` overlay
activation is **explicitly out of scope / deferred** to a separate follow-on issue
(open it after the sim + on-water re-test of the 1.8 anchor).
