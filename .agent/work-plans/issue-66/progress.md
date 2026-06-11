---
issue: 66
---

# Issue #66 — Cross-track controller over-corrects into 360° loops on planner paths (yaw-ceiling × path-jump synergy)

## Plan Authored
**Status**: complete
**When**: 2026-06-10 23:39 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-66/plan.md` at `f664363`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/75 (`[PLAN]` prefix)
**Phases**: single (A reference-stabilization + B controller-robustness + tests; splittable)

### Open questions
- [ ] Slew-limit the cross-track *error* (preserves PID semantics) vs the *output* yaw — recommend error-side
- [ ] Derivative-on-measurement term: enable this PR vs land disabled (default 0) pending field tuning — recommend disabled
- [ ] One cohesive PR (A+B+tests) vs split A→B if review prefers smaller diffs

## Plan Review
**Status**: complete
**When**: 2026-06-11 05:44 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-66/plan.md` at `f664363`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/75
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) #66 fix-direction (2) — survey yaw-rate **magnitude** cap, decoupled from the 1.0 rad/s capability ceiling — absent (slew-limit is rate-of-change, not magnitude); add or explicitly defer — plan.md Front B
- [ ] (must-fix) Cross-repo consequence: deployed controller params live in unh_echoboats_project11 (bizzyboat.yaml / nav2_overlay.yaml), not the nav pkg — add to Consequences — plan.md Consequences
- [ ] (suggestion) Scope: 3 components / 2 repos — commit to A→B stacked split — plan.md Open Questions/Estimated Scope
- [ ] (suggestion) Name the nav2 BT replan-hysteresis mechanism (IsPathValid + goal-unchanged gate, or RateController) — plan.md Front A.2
