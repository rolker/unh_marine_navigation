---
issue: 46
---

# Issue #46 — Hover sticks to first-evoked location — SequenceWithMemory never re-ticks PredictStoppingPose on re-entry

## Plan Authored
**Status**: complete
**When**: 2026-05-29 10:01 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-46/plan.md` at `9f20785`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/47 (`[PLAN]` prefix)
**Phases**: 2 (Phase 1 = node swap + regression test this PR; Phase 2 = reactive re-command, stack pending decision)

### Open questions
- [ ] Bundle Phase 2 (same-type re-command reactive fix) here or stack as a follow-up? Recommend stacking.
- [ ] Phase 2 mechanism: inline reactive id-latch in BT XML vs. a dedicated custom condition node?
- [ ] Should `done_hover` carry the last survey pose explicitly instead of relying on drift projection? (Leaning no.)
