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
- [x] Host shape → **decorator wrapper** (inner CrabbingPathFollower)
- [x] Old BT node → **delete now** (registration + plugin_lib_names + tests)
- [x] Package name → **marine_nav_avoidance_controller**
- [x] Timing → **all before June 4** (plugin + config-flip), on-water validation before June 15
- [ ] Mid-line re-send (#35): verify new-goal setPlan() propagates through the wrapper (in-code)
