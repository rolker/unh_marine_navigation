---
issue: 59
---

# Issue #59 — Move survey-line obstacle avoidance into a controller decorator (off the BT node)

## Plan Authored
**Status**: complete
**When**: 2026-06-02 10:12 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-59/plan.md` at `fd23ac8`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/60 (`[PLAN]` prefix)
**Phases**: single core PR + two follow-up cross-repo config PRs (echoboats, seafloor)

### Open questions
- [ ] Decorator wrapper vs. folding avoidance into CrabbingPathFollower (lean: decorator)
- [ ] Delete the AdjustPathForObstacles BT node now, or keep deprecated one cycle?
- [ ] New package name: marine_nav_avoidance_controller?
- [ ] Mid-line re-send (#35): verify new-goal setPlan() propagates through the wrapper
- [ ] Cross-repo config rollout cadence vs. June 4 dev freeze
