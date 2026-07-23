---
issue: 99
---

# Issue #99 — CrabbingPathFollower: segment cursor preserved by index across same-goal setPlan re-issues — dense→sparse flip skips to final leg

## Plan Authored
**Status**: complete
**When**: 2026-07-22 14:54 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Plan**: `.agent/work-plans/issue-99/plan.md` at `af355c7`
**Branch**: feature/issue-99 at `af355c7`
**Phases**: single

### Open questions
- [ ] Tie-break strategy for ambiguous nearest-point candidates (arc-fraction tie-break vs bounded forward-only window)
- [ ] AvoidanceController always-resample companion: separate issue now, or after sim soak?

## Plan Review
**Status**: complete
**When**: 2026-07-22 15:41 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-99/plan.md` at `af355c7`
**PR**: PR-less (--issue / file-path mode)
**Verdict**: approve-with-suggestions

Premise verified against source: `AvoidanceController::reshapeAroundObstacles`
returns the sparse `nominal_plan_` when `peak < kDeviationEpsilon` (and on the
no-costmap / <3-station / blocked-corridor fallbacks) and the dense n-station
2 m reshape otherwise — the exact sparse↔dense same-goal alternation the issue
describes. The `min(index, segment_count−1)` cap at `crabbing_path_follower.cpp:710`
resolves a dense index onto the sparse final leg as claimed. File targeting is
exact; scope (4 files) is single-PR; consequences table, ADR mapping (0008/0018),
and principles self-check are sound. Findings are refinements, not structural gaps.

### Findings
- [ ] (suggestion — near must-fix) Self-crossing / tightly-spaced parallel-leg robustness: global nearest-point over all new segments + arc-fraction tie-break can mis-map a laterally-offset reshape onto an ADJACENT parallel leg once the offset approaches half the leg spacing. The plan's own Open Question flags this; resolve it toward a bounded forward-only window anchored on the previous cursor's arc-length/index (the issue's stated alternative), or a hybrid, and make the ≤6 m-corridor test adversarial (offset ≥ half the zigzag leg spacing) so it exercises the ambiguity instead of passing trivially — `plan.md:28-29`, `plan.md:39`, `plan.md:88`
- [ ] (suggestion) Test-bullet-5 reference-step bound is mis-stated: it cites "the #66 slew absorption for one 5 Hz cycle (0.6 m at the configured 3 m/s)", but `cross_track_error_slew_rate` is a separate parameter defaulting to 0.0 (limiter OFF), not the vehicle speed — per-cycle absorption is `slew_rate·dt` and is zero by default; the controller runs ~10 Hz (project AGENTS.md), not 5 Hz. Reframe the assertion geometrically (the cursor mapping itself bounds the cross-track reference step), since the fix must not lean on a default-off slew limiter as the safety net — `plan.md:44-46`
- [ ] (suggestion) Degenerate new-path guard: `mapCursorToNewPath` should defensively handle empty / single-pose new paths (floor via `std::max(0, segment_count−1)`, mirroring the existing cap and the empty/single-point guards throughout `path_geometry.hpp`), with a named test case — `plan.md:36-48`
- [ ] (observation) Anchor granularity: anchoring at the old current segment's START point (not the boat's projected position) means sparse→dense flips re-localize to the leg start and rely on `computeVelocityCommands`' forward scan (`crabbing_path_follower.cpp:794-843`) to re-advance within the same cycle — safe (conservative, never overshoots, pose-based + monotone) but the plan should note this interaction so the implementer preserves the compute-side localizer — `plan.md:21-33`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-23 08:09 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-99 at `46cb847`
**Mode**: pre-push
**Depth**: Deep (reason: 689 changed lines >200; safety-critical nav-control path)
**Must-fix**: 0 | **Suggestions**: 1
**Round**: 1 | **Ship**: recommended — no must-fix; core algorithm verified correct and the gated gtest suite passes

Static analysis: cppcheck clean on new files (2 findings on `crabbing_path_follower.cpp` are on
pre-existing untouched lines, out of scope). cpplint/copyright/uncrustify failures are pre-existing
package-wide debt (verified against `origin/jazzy`), tracked in `unh_marine_navigation#98` and
excluded from the hosted CI gate (`ci.yml` `-LE linter` / `-m "not linter"`); the new file rides the
same package convention. Governance: ADR-0008 (controller contract unchanged, no API/param change) —
compliant; consequences (06-04 comment rewritten, cap branch removed not left dead, gabby rebuild
noted for PR body) — addressed. Plan drift: none — all 4 planned files changed, every named test case
present, all three prior plan-review findings folded in. Adversarial Lens A (logic) + Lens B
(systemic/lifecycle) run in-context/sequential (no fresh-subagent dispatch available in this
container): return value provably never the done-sentinel; first-plan path avoids empty-`global_plan_`
UB; single-threaded state model unchanged; anchor-at-segment-start relies safely on the compute-side
forward scan. **Build + test executed**: `test_plan_cursor` gtest PASSED (all cases), all other gated
gtest suites PASSED.

### Findings
- [ ] (suggestion) `test/test_plan_cursor.cpp`: add direct `#include <limits>` (`std::numeric_limits`) and `#include <algorithm>` (`std::max`/`std::clamp`) — currently transitive via `path_geometry.hpp` (IWYU); minor, and cpplint's IWYU check rides the CI-excluded `linter` label — `test_plan_cursor.cpp:11-13`
- [ ] (note) Adjacent-parallel-leg rejection has a documented boundary: a lateral reshape whose displacement approaches leg spacing can still fall to the global fallback and capture a neighbor. Acknowledged in code/plan; realistic-boundary test `AdjacentParallelLegNotCaptured` passes. Acceptable tradeoff, not a defect — `path_geometry.hpp:335-341`
- [ ] (note) copyright/cpplint/uncrustify lint tests fail but are pre-existing package-wide debt (`unh_marine_navigation#98`), excluded from the CI gate; not a regression from this PR
