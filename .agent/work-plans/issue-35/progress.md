---
issue: 35
---

# Issue #35 — Mission re-send mid-line doesn't take effect — BT latches FollowPath path on same-type task switch

## Issue Review
**Status**: complete
**When**: 2026-05-26 14:18 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #35
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/35#issuecomment-4550278746
**Scope verdict**: well-scoped

### Actions
- [ ] A change includes its consequences / Test what breaks: add a regression test reproducing the N→N+1 same-type mid-line interrupt; coordinate with #7 / #8 (no BT-navigator test harness exists yet).
- [ ] Surface the behavioral choice before implementing: identity-gated re-entry vs. goal preempt/cancel (and whether the new line restarts from a fresh transit-to-start) is operator-facing — confirm intended interrupt semantics first.
- [ ] Make the preemption observable (log the `FollowPath` halt/restart) so reported task and executed path stay coupled.
- [ ] Sequence with #25 / #28 — all three edit `run_tasks.xml`.
- [ ] Capture the fix-direction rationale in the PR (no project ADR system exists).
