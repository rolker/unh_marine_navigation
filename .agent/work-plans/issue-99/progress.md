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
