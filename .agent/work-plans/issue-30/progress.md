---
issue: 30
---

# Issue #30 — Decompose tracklines into short, skippable segments (partial coverage-planner port)

## Plan Authored
**Status**: complete
**When**: 2026-05-31 22:50 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-30/plan.md` at `fd32bc9`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/51 (`[PLAN]` prefix)
**Phases**: single

> Note: plan supersedes the issue-body design (segment decomposition → reactive
> Frenet corridor path-adjuster BT node). Issue text needs human reconciliation.

### Open questions
- [ ] max_xte source — per-line task field (survey spec) vs node param vs crab-follower XTE config
- [ ] Confirm survey-line pose frame == map_tide (else TF into costmap frame)
- [ ] Confirm blackboard "node" is spun so a cached costmap subscription fires (else callback group / poll)
- [ ] Inflation-tail handling — g(cost) keys off inscribed/obstacle+sea_surface bands, not 150 m inflation tails
