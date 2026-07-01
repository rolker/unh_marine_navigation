---
issue: 89
---

# Issue #89 — Anticipatory speed regulation via path curvature

## Issue Review
**Status**: complete
**When**: 2026-07-01 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #89
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

Adds a `curvatureSpeedFactor()` pure function to `path_geometry.hpp`, new
`turn_curvature_min_radius` param (and companion `_range`), integration into
`computeVelocityCommands()` via `min()` with the existing crab-angle factor,
binding in `bindCrabbingControls()` / `declareCrabbingControlParams()`, and
unit tests. This is the same shape and size as #87 — a single focused PR in
`marine_nav_crabbing_path_follower`. No cross-repo or interface changes needed.

**Right repo?** `unh_marine_navigation` (`marine_nav_crabbing_path_follower` package) — correct.

**Dependencies**: Depends on #87/#88 (already shipped). Compose-with-#32
(speed precedence) is covered in the issue description.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Default `turn_curvature_min_radius = 0.0` is a clean no-op; live-settable via marine_control; same opt-in idiom as existing params |
| A change includes its consequences | Watch | `declareCrabbingControlParams()` and `bindCrabbingControls()` must both be updated; issue implies this but does not call it out explicitly |
| Test what breaks | OK | Issue lists concrete unit-test cases: straight path → 1.0; constant-radius arc; degenerate inputs (<3 pts, collinear, NaN); floor respected — mirrors the `turnSpeedFactor` test structure |
| Only what's needed | OK | Reuses existing lookahead machinery; no new lookahead horizon introduced |
| Improve incrementally | OK | Directly follows #87; default-off maintains backward compatibility |
| Safety First (project) | OK | Degenerate-input contract (collinear → radius infinity → factor 1.0; <3 pts → 1.0; NaN → 1.0; non-finite `linear.x` never emitted) is explicitly stated and comprehensive |
| Hardware agnosticism (project) | OK | Pure path-geometry computation; no platform-specific APIs |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 — Worktree isolation | Yes | Already in worktree `issue-unh_marine_navigation-89` ✓ |
| 0008 — ROS 2 conventions | Yes | New param with FloatingPointRange descriptor + `_range` companion follows established pattern ✓ |
| 0013 — progress.md vocabulary | Yes | This entry ✓ |

### Consequences

- `declareCrabbingControlParams()` must declare the new param(s).
- `bindCrabbingControls()` must bind them to the marine_control panel (speed group).
- Unit test file for `curvatureSpeedFactor` should be added alongside `test_turn_speed_factor.cpp`.

### Recommendations

1. **Curvature algorithm choice**: The issue offers two options (3-point circumscribed-circle fit
   vs. heading-change-per-arc-length). On piecewise-linear paths both are valid but the
   circumfit is more direct and already has a clean degenerate case (collinear → inf radius → factor
   1.0). Recommend settling this in the plan rather than leaving it open for implementation.

2. **Param naming consistency**: Existing params are `turn_speed_max_crab_deg` /
   `turn_speed_min_factor`. The new param should be `turn_speed_curvature_min_radius` (not
   `turn_curvature_min_radius`) to stay within the existing `turn_speed_*` namespace group.

3. **Shared or separate min-factor floor**: It is unspecified whether the curvature factor
   reuses `turn_speed_min_factor_` (the crab-angle floor) or gets its own
   `turn_speed_curvature_min_factor_`. Since the two regulators now compose via `min()`, sharing
   the floor is simpler and avoids the operator having to set two separate floors; a separate
   param is only warranted if the desired floors differ. Clarify in the plan.

### Actions
- [ ] Resolve curvature algorithm (3-point circumfit recommended) in the work plan.
- [ ] Confirm param naming: prefer `turn_speed_curvature_min_radius` for namespace consistency.
- [ ] Clarify whether curvature regulation shares `turn_speed_min_factor_` or introduces its own floor param.

## Plan Authored
**Status**: complete
**When**: 2026-07-01 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-89/plan.md` at `256e2f2`
**Branch**: feature/issue-89 at `256e2f2`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-01 03:53 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-89/plan.md` at `256e2f2`
**PR**: PR-less (--issue mode)
**Verdict**: approve-with-suggestions

Independent review (fresh-context sub-agent, distinct model from the Sonnet
plan author). All three `review-issue` findings verified as addressed:
curvature algorithm settled (3-point circumfit), param renamed to
`turn_speed_curvature_min_radius`, floor reuses shared `turn_speed_min_factor_`.
All structural claims cross-checked against source: `kTunables` loop drives both
`declareCrabbingControlParams`/`bindCrabbingControls` (auto-pickup confirmed),
integration site matches the `turnSpeedFactor` pattern, count assertions at
`test_crabbing_control.cpp:119,284` (12u→13u), ADRs 0002/0008/0013 present.

### Findings
- [ ] (suggestion) Circumfit uses the boat's actual position (`pose_in_plan.pose.position`), which carries cross-track error, as the first fit point while the other two are on-path; consider the along-track projection or document the choice — `plan.md:76-80`
- [ ] (suggestion) Add a coincident-point test case (near-goal: half_la == full_la == goal → R = ∞ → factor 1.0) — the planned 8 cases omit it — `plan.md:88-96`

## Implementation
**Status**: complete
**When**: 2026-07-01 04:10 +00:00
**By**: Claude Opus

**Plan**: `.agent/work-plans/issue-89/plan.md`
**Branch**: feature/issue-89
**Verdict**: implemented per plan; both Plan Review suggestions applied (mandatory)

### What was implemented
- `path_geometry.hpp`: added `circumscribedRadius(a, b, c)` (3-point
  circumscribed-circle radius; returns `+inf` for collinear / coincident /
  non-finite inputs via a single `!isfinite` guard on the result) and
  `curvatureSpeedFactor(radius, min_radius, min_factor)` (disabled when
  `min_radius <= 0`; 1.0 for non-finite/gentle radius; else
  `clamp(radius/min_radius, min_factor, 1.0)`). Added `#include <limits>`.
- `crabbing_path_follower.h`: added `turn_speed_curvature_min_radius_`
  `std::atomic<double>{0.0}` with doc comment.
- `crabbing_path_follower.cpp`: added the `kTunables[]` entry
  (`turn_speed_curvature_min_radius`, default 0.0, range `[0, 200]`, units `m`,
  group `speed`) — control count 12→13; added `read_validated` call
  (`lo=0.0, exclusive_lo=false`); added the on-set-parameters callback branch
  (`>= 0.0`, no upper bound); applied the factor at the turn-speed regulation
  site as `combined_factor = min(turn_factor, curvature_factor)` before the
  `cos_crab` division; reuses `turn_speed_min_factor_` as the shared floor.
  Updated "eleven"→"twelve" tunable-count comments.
- New `test/test_curvature_speed_factor.cpp` (9 gtest cases) wired into
  `CMakeLists.txt` like `test_turn_speed_factor`.
- `test/test_crabbing_control.cpp`: bumped both size assertions 12u→13u,
  updated "twelve"→"thirteen" / "nine"→"twelve" doc comments, added
  group/units/range assertions (`speed`, `m`, `[0, 200]`) for the new control.
  Also updated the CMake comment and header "eleven"→"twelve".

### Along-track fit-point decision + Plan Review suggestions (both applied)
- **Sug 1 (along-track fit point):** the first circumfit point is the along-track
  on-path projection — `lookaheadPoint(poses, current_segment_, progress, 0.0)`
  (the projected foot on the current segment), NOT `pose_in_plan.pose.position`.
  The other two points are the half-lookahead and full-lookahead points (the
  full-lookahead `la_point` was hoisted out of the `if (lookahead > 0.0)` block
  and reused). All three fit points are path-referenced; a code comment at the
  site states this, so curvature stays a pure path property and cross-track error
  does not double-count with the crab-angle regulator.
- **Sug 2 (coincident-point test):** added
  `CircumscribedRadius.CoincidentPointsAreInfinite` covering all-three-coincident
  and near-goal half==full==goal → R = ∞ → factor 1.0.

### Control-count bump (#87 lesson)
Advertised control set grew 12→13. Updated `test_crabbing_control.cpp` size
assertions (`:119`, `:284`) and doc comments, added the new control's
group/units/range assertions, and updated the CMake and header count comments.

### Build & test (ACTUAL)
- Build: `colcon build --packages-up-to marine_nav_crabbing_path_follower
  --symlink-install` → **4 packages finished**, `marine_nav_crabbing_path_follower`
  compiled (only pre-existing `-Wunused-parameter` / `-Wsign-compare` warnings in
  untouched code).
- Tests: `colcon test --packages-select marine_nav_crabbing_path_follower`.
  Functional gtest suites ALL PASS:
  - `test_curvature_speed_factor`: 9/9
  - `test_crabbing_control`: 10/10
  - `test_turn_speed_factor`: 7/7
  - `test_gain_schedule`: 9/9
  - `test_path_geometry`: 18/18
  (53 functional tests, 0 failures, 0 errors.)
- Pre-existing package-wide lint remains (NOT fixed here, per instructions):
  cpplint `legal/copyright` + `build/include_order` and uncrustify style diffs.
  Confirmed pre-existing — `ament_uncrustify src/crabbing_path_follower.cpp`
  fails on the pristine (pre-change) file, and the copyright/include-order
  findings hit every sibling test file (`test_turn_speed_factor.cpp`,
  `test_gain_schedule.cpp`, `test_path_geometry.cpp`).

### Deviations from plan.md
- First circumfit point is the along-track projection, not
  `pose_in_plan.pose.position` (Plan Review sug 1, operator-mandated).
- Curvature test file has 9 cases (planned 8) — the added coincident-point case
  (Plan Review sug 2).
- DEBUG log prints `turn_factor` / `curvature_factor` / `combined_factor`.
- No other deviations.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-01 04:22 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-89 at `9ebcf68`
**Mode**: pre-push
**Depth**: Deep (reason: 200+ changed lines; real-time vessel motion-control)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix; math/degenerate-handling verified safe, follows the merged #87 pattern, no-op until a platform opts in.

### Findings
- [x] (suggestion) No end-to-end test exercises the curvature wiring (foot/half/full point selection + `min()` composition) through a multi-segment `global_plan_` — `src/crabbing_path_follower.cpp:991-1006`
- [x] (suggestion) Shared `turn_speed_min_factor` floor couples both regulators; note it in the param help text — `include/marine_nav_crabbing_path_follower/path_geometry.hpp:243-244`

### Notes
- Static analysis: cppcheck clean on new code; ament_cpplint findings (copyright, include-order, line-length) are pre-existing repo convention, mirrored from the merged sibling `test_turn_speed_factor.cpp` (#87), not a gating check. Silence-filtered.
- Dropped one Lens B false positive: the `circumscribedRadius` doc comment (path_geometry.hpp:189 vs 216) is internally consistent (`|cross| = 2·Area`), not contradictory.
- Two disjoint-lens Claude Adversarial passes (A: logic/correctness; B: systemic/safety) both returned zero must-fix.

## Implementation
**Status**: complete
**When**: 2026-07-01 04:32 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-89 at `a61857d`
**Addressed**: Local Review (Pre-Push) — When 2026-07-01 04:22 +00:00, branch `9ebcf68` (0 must-fix, 2 suggestions)
**Commits**: `5e975d8` (test), `a61857d` (help text)

### Actions
- [x] (suggestion) End-to-end test of the curvature wiring over a multi-segment `global_plan_` — `marine_nav_crabbing_path_follower/test/test_curvature_speed_factor.cpp`. Added a `CurvatureWiring` suite (6 cases) that mirrors the composition at `crabbing_path_follower.cpp:991-1006`: a `regulate()` helper selects the along-track foot / half-lookahead / full-lookahead points off a real multi-segment `PoseStamped` plan via `lookaheadPoint()`, feeds them to `circumscribedRadius`/`curvatureSpeedFactor`, and composes with the reactive `turnSpeedFactor` via `min()`. Covers: straight plan → no curvature slowdown; L-shaped corner → radius exactly 12.5 m → factor 0.5 (pins fit-point selection + geometry); `progress`>0 shifts the foot mid-segment; `min()` composition with either regulator governing; and the disabled (`curvature_min_radius`=0) no-op on a curved plan. Kept at the pure-function composition level (not a full ROS-lifecycle `calculate()` harness — none exists in this package and one would be disproportionate for a suggestion); the helper carries a comment requiring it be kept in lockstep with the call site.
- [x] (suggestion) Noted the shared `turn_speed_min_factor` floor in the param help text — `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp:65-68`. The `kTunables` description now states the floor applies to BOTH turn-speed regulators (the reactive `turn_speed_max_crab_deg` one and the anticipatory `turn_speed_curvature_min_radius` one), matching the internal doc-comment note at `path_geometry.hpp:243-244`.

### Build & test (ACTUAL)
- `colcon build --packages-select marine_nav_crabbing_path_follower --symlink-install` → 1 package finished (only pre-existing `-Wunused-parameter`/`-Wsign-compare` warnings in untouched code).
- `colcon test --packages-select marine_nav_crabbing_path_follower`: functional gtest suites ALL PASS — `test_curvature_speed_factor` now 15/15 (9 unit + 6 new wiring), `test_crabbing_control` 10/10, others unchanged. Pre-existing package-wide lint (cpplint copyright/include-order/line-length + uncrustify) remains, unchanged from the reviewed diff — documented as non-gating repo convention mirrored from sibling test files (#87), NOT introduced here.

### Next step
Lifecycle: Implementation → review-code (re-review the fixes). Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 89 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-01 04:41 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-89 at `88edf1d`
**Mode**: pre-push
**Depth**: Deep (reason: real-time vessel motion-control law; 200+ changed code+test lines)
**Must-fix**: 0 | **Suggestions**: 1
**Round**: 2 | **Ship**: recommended — round-1's 2 suggestions addressed (wiring suite + shared-floor help text); no must-fix this round; both Deep adversarial lenses returned zero.

### Findings
- [ ] (suggestion) Test comment cites call-site range `crabbing_path_follower.cpp:991-1006`; the `min()` composition it mirrors now lands at `1007-1008` — nudge the range to keep the lockstep pointer exact — `test/test_curvature_speed_factor.cpp:37,209`

### Notes
- Round-2 delta since round 1 (`9ebcf68`) is exactly the two addressed pre-push suggestions: the `CurvatureWiring` end-to-end suite (6 cases) and the shared-floor help text. Core algorithm unchanged since round-1 approval.
- Two disjoint-lens Claude Adversarial passes (A: logic/correctness; B: systemic/safety) both returned zero must-fix. Verified: circumradius formula correct ((0,0),(15,0),(20,10)→R=12.5); no non-finite surge path (degenerate → `+inf` → factor 1.0); `combined_factor = min(...) ≤ 1.0` (boat can only slow); atomics `.load()`-snapshotted tear-free.
- Dropped Lens B false positive (callback already uses `if (!as_number(p, v) || !update(...))`) and a defensive-only isfinite-guard (provably unreachable — target_speed finite, factor in [min,1.0]).
- Static analysis: cppcheck findings (`cos_error_azimuth`:779, `dt` shadow:952) are on pre-existing untouched lines — silence-filtered. cpplint copyright/include-order on the new test file is repo-wide convention mirrored from the merged #87 sibling tests, non-gating.

### Next step
Lifecycle: **Local Review (approved)** → push / open PR → **triage-reviews**. Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 89 --skill triage-reviews
