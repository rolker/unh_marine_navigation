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
