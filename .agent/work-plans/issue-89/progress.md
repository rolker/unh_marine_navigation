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
