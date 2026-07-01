---
issue: 91
---

# Issue #91 — Fix look-ahead double-correction in CrabbingPathFollower

## Issue Review
**Status**: complete
**When**: 2026-07-01 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #91
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope
Single bug fix in `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp`
(lines 930–932): replace the boat-to-look-ahead-point bearing with the segment
azimuth at the look-ahead point. The fix is confined to one controller and one
logical change. Acceptance criteria include a unit test and a sim/log check — both
are clearly bounded. Single PR.

**Right repo?** Yes — `unh_marine_navigation` project repo, `marine_nav_crabbing_path_follower` package.

**Dependencies**: #90 (merged) is a logical predecessor (curvature anticipation is
gated on stable look-ahead). No blocking dependency on open issues.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix is clearly motivated; existing DEBUG log at line 897 continues to expose `base_heading` and `crab_angle`. Note: that log fires *before* the look-ahead block so after the fix `base_heading` in the log will reflect the updated look-ahead azimuth — confirm it still makes sense. |
| A change includes its consequences | Watch | Issue specifies a unit test for `base_heading` derivation and a sim/log check. The implementation will need a new pure function in `path_geometry.hpp` (e.g. `lookaheadSegmentAzimuth`) to retrieve the azimuth of the segment the look-ahead point lands on — `lookaheadPoint()` returns only a `Point`, not the segment index. The new function must be unit-tested alongside it. |
| Test what breaks | OK | Acceptance criteria explicitly require a unit test for the `base_heading` derivation and a monotonic cross-track error sim check. Existing test patterns in `test_path_geometry.cpp` are directly reusable. |
| Improve incrementally | OK | Narrow, targeted fix. Does not touch the crab PID, speed regulation, or curvature logic. |
| Only what's needed | OK | Fix is minimal. One new helper function in `path_geometry.hpp`, one changed call site in `crabbing_path_follower.cpp`. |
| Capture decisions, not just implementations | OK | The issue body itself is the decision record (root cause, rejected alternative, rationale). No ADR needed for a bug fix. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0008 — ROS 2 conventions | Yes | Fix touches a Nav2 controller plugin; follow ROS 2 Jazzy/Rolling conventions. No deviation expected. |
| ADR-0013 — progress.md vocabulary | Yes | This entry uses `## Issue Review` per ADR-0013. |
| Others | No | No workspace infra, Python packaging, build system, or agent-instruction changes. |

### Consequences

- `path_geometry.hpp` will likely gain a new function (`lookaheadSegmentAzimuth` or
  a combined `lookaheadPointAndSegment`). Corresponding tests should be added to
  `test_path_geometry.cpp`.
- The DEBUG log at line 897 logs `segment_azimuth` before the look-ahead block.
  After the fix the logged value will be the look-ahead segment azimuth (correct),
  but the label in the log message should be verified to still be accurate.
- No message-type or interface changes → no downstream package updates needed.

### Recommendations

- Define a pure `lookaheadSegmentAzimuth(poses, start_seg, start_offset, lookahead)`
  (or equivalent) in `path_geometry.hpp` rather than inlining the segment-walk
  logic a second time in `crabbing_path_follower.cpp`. Keeps the geometry layer
  testable in isolation, consistent with the existing `lookaheadPoint` pattern.
- Update the DEBUG log string at line 897 if needed so the label matches the
  semantics of the value being logged (look-ahead segment azimuth vs. current
  segment azimuth).

### Actions
- [ ] Add `lookaheadSegmentAzimuth` (or combined `lookaheadPointAndSegment`) to `path_geometry.hpp` and unit-test it in `test_path_geometry.cpp`.
- [ ] Verify/update DEBUG log string at line 897 after changing the `base_heading` computation.

## Plan Authored
**Status**: complete
**When**: 2026-07-01 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-91/plan.md` at `3c60486`
**Branch**: feature/issue-91 at `3c60486`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-01 11:53 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-91/plan.md` at `3c60486`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) `lookaheadSegmentAzimuth` should return the final segment's azimuth when `start_seg` is past the last segment (mirror `lookaheadPoint`'s goal-clamp, `path_geometry.hpp:82-84`), not fold it into the `0.0`-degenerate bucket — add a parallel out-of-range test. — `plan.md:47`
- [ ] (suggestion) Note in the updated comment block that the segment-tangent `base_heading` steps discretely at vertex crossings (bounded by the `max_yaw_rate` clamp) — `plan.md:38`
- [ ] (suggestion) Confirm the sim/log monotonic cross-track decay acceptance check is run manually at review-code (it's not an automated deliverable). — `plan.md:66`
- [ ] (info) Plan correctly determined the DEBUG log at `crabbing_path_follower.cpp:897` needs no change — it logs `segment_azimuth` (current segment, line 815), not `base_heading`; the review-issue log action item is discharged.

## Implementation
**Status**: complete
**When**: 2026-07-01 13:00 +00:00
**By**: Claude Opus

**Branch**: feature/issue-91
**Plan**: `.agent/work-plans/issue-91/plan.md`

### What changed
- **`path_geometry.hpp`** — added pure inline `lookaheadSegmentAzimuth(poses, start_seg, start_offset, lookahead) → double`. Walks the path with the SAME forward-only traversal as `lookaheadPoint` and returns the azimuth (`atan2(dy, dx)`, radians) of the segment CONTAINING the look-ahead point.
- **`src/crabbing_path_follower.cpp`** — replaced the boat-to-look-ahead-point `atan2` bearing (`~:930-932`) with `base_heading = AngleRadians(lookaheadSegmentAzimuth(global_plan_.poses, current_segment_, progress, lookahead));`. The preceding `lookaheadPoint(...)` call and `la_point` are KEPT — the #90 curvature block reuses `la_point` as its third circumfit point. Updated the comment block (`~:899`) from "pure-pursuit bearing to a point" to "path tangent at the look-ahead point", explaining the no-double-count rationale.
- **`test/test_path_geometry.cpp`** — 4 new `LookaheadSegmentAzimuth` tests.

### Plan Review suggestions applied (all 3 mandatory)
1. **Past-end / out-of-range clamp → FINAL segment azimuth** (not `0.0`): implemented via a `start_seg > last - 1` early return AND the loop's `i == last - 1` fallthrough, mirroring `lookaheadPoint`'s goal-clamp. Only empty path, single point, and all-coincident/zero-length inputs return `0.0` (`atan2(0,0)` is naturally 0). Covered by `ClampsToFinalSegmentPastEnd` (both horizon-overshoot and past-end `start_seg`).
2. **Comment note on discrete vertex-crossing steps**: the updated comment states `base_heading` steps discretely as the look-ahead point crosses a vertex, bounded downstream by the `max_yaw_rate` clamp.
3. **Manual sim check owed at review-code** — see below.

### Tests (`LookaheadSegmentAzimuth`)
- `StraightLineIsBoatPositionIndependent` (the AC test — function takes no boat position, returns segment azimuth).
- `AnticipatesNextSegmentPastBend` (past a vertex → next segment azimuth; short horizon → current).
- `ClampsToFinalSegmentPastEnd` (overshoot and past-end start both → final segment azimuth).
- `HandlesDegeneratePaths` (empty + single-point → 0.0, no crash).

### ⚠ Manual verification owed at review-code
Per Plan Review sug 3, the simulator was NOT run in this container. At **review-code**, run the sim with `lookahead_time` = 0 vs 1–2 s and confirm cross-track error decays **monotonically** (no growing oscillation) with look-ahead enabled. This is a manual review-code deliverable, not an automated test.

### Build / test result (ACTUAL)
- Build: `colcon build --packages-up-to marine_nav_crabbing_path_follower --symlink-install` → **succeeded** (used the fallback; `build.sh` alone failed because underlay deps weren't yet built in this worktree). Only pre-existing unused-parameter / sign-compare warnings.
- Test: `colcon test --packages-select marine_nav_crabbing_path_follower` + `colcon test-result --verbose`.
  - **All functional gtest suites PASS**: `test_path_geometry` **22/22** (18 prior + 4 new), `test_crabbing_control` 10/10, `test_curvature_speed_factor` 15/15, `test_gain_schedule` 9/9, `test_turn_speed_factor` 7/7 — **0 failures, 0 errors** across all gtests.
  - The 51 `colcon test-result` failures are **pre-existing package-wide lint only** (cpplint `legal/copyright`, `build/include_order`; uncrustify on `crabbing_path_follower.{h,cpp}`) — noted, not fixed per instructions.

### Deviations from plan.md
None. Approach and all four planned tests implemented as specified, with the 3 Plan Review suggestions folded in.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-01 13:53 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-91 at `acf8bea`
**Mode**: pre-push
**Depth**: Deep (reason: 344 total changed lines >200 + safety-relevant control-law change; code portion alone is Standard-sized)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 1 | **Ship**: recommended — no must-fix findings; clean, well-scoped control-law fix, 22/22 gtests re-verified locally, static analysis clean on changed lines.

### Findings
- [x] (suggestion) Comment at `:905-907` credits the `max_yaw_rate` clamp as the discrete-step bound, but at default params it is a no-op — the external `velocity_smoother` is the real bound (see `crabbing_path_follower.h:108-109`) — `src/crabbing_path_follower.cpp:905`
- [x] (suggestion) Segment-tangent `base_heading` can toggle across a vertex cycle-to-cycle on dense/short-segment plans (chatter); no hysteresis and no worst-case (hairpin) test — `include/marine_nav_crabbing_path_follower/path_geometry.hpp:143`
- [x] (suggestion) Mixed-reference bend regime: `base_heading` (ahead segment) + `crab_angle` (current segment) superposed; convergence provable only in the straight regime; transition untested — `src/crabbing_path_follower.cpp:936`
- [x] (suggestion/owed) Sim/log monotonic cross-track decay acceptance check still unverified — could not run in offline container (no simulator); owed before merge — `n/a` (deferred: no simulator in this offline container — a manual review-code/merge deliverable, not addressable via code change; still owed before merge)

### Notes
- Lens A (logic/correctness): no findings — traced `lookaheadSegmentAzimuth` against `lookaheadPoint` line-by-line (byte-identical traversal → same segment landing), independently verified all 4 test assertions, no boundary/OOB defects.
- Static analysis: cpplint + cppcheck clean on all changed lines; new function has zero findings. Pre-existing package-wide lint (legal/copyright, line-length) exists on untouched lines — not attributable to this PR.
- Plan drift: none. Governance: all consequences addressed; ADR-0008/0013 compliant.

## Implementation
**Status**: complete
**When**: 2026-07-01 14:09 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-91 at `fc424d1`
**Addressed**: `## Local Review (Pre-Push)` (When 2026-07-01 13:53 +00:00, at `acf8bea`)
**Commits**: `178d069`, `de0182c`, `fc424d1`

Consumed the four unchecked suggestions from the pre-push review. Three fixed via
code/doc/test changes; one deferred (no simulator offline). All 5 gtest suites
re-verified locally after the changes: `test_path_geometry` 23/23 (22 prior + 1
new hairpin), `test_crabbing_control` 10/10, `test_curvature_speed_factor` 15/15,
`test_gain_schedule` 9/9, `test_turn_speed_factor` 7/7 — 0 failures. Build clean
(only pre-existing unused-parameter / sign-compare warnings on untouched lines).

### Actions
- [x] Correct discrete-step rate-bound comment: `max_yaw_rate_` defaults to ±pi rad/s (~180°/s) so the controller-level clamp is effectively a no-op at default params; the external `velocity_smoother` is the real rate limiter. Comment rewritten to credit it, cross-referencing `crabbing_path_follower.h:107-111` — `src/crabbing_path_follower.cpp:905` (`178d069`)
- [x] Add worst-case (hairpin) test `HairpinReversalStepsByNearlyPi` characterizing the near-π single-cycle `base_heading` step at a ~180° reversal, and documenting that this pure geometry helper deliberately has no hysteresis (smoothing is the downstream `velocity_smoother`'s job) — `test/test_path_geometry.cpp` (`de0182c`). Hysteresis itself is a separate design change with its own state/tuning, out of scope for the #91 bug fix (deferred to a follow-up).
- [x] Document the mixed-reference bend-regime limitation at the `target_heading = base_heading + crab_angle` superposition: proven-convergent only in the straight (same-segment) regime; the bend transition is a bounded, self-correcting heuristic verified by the owed sim check, not a unit test — `src/crabbing_path_follower.cpp:936` (`fc424d1`)
- [x] Sim/log monotonic cross-track decay acceptance check — `n/a` (deferred: no simulator in this offline container; a manual review-code/merge deliverable, not a code change — still owed before merge, as the review itself flagged)
