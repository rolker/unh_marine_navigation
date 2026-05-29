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

## Plan Review
**Status**: complete
**When**: 2026-05-29 10:09 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — independent fresh-context sub-agent review (not author self-review)

**Plan**: `.agent/work-plans/issue-46/plan.md` at `68d0971`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/47
**Verdict**: changes-requested → all must-fix addressed inline (plan re-committed at `68d0971`)

### Findings
- [x] (must-fix) `halt()`-reset claim was stated as fact though unreadable from source — softened prose, noted it is pinned by the Phase-1 gtest. — `plan.md` Context
- [x] (must-fix) Continuous-hold no-recompute path was unpinned; added test arm (b) tick→tick→assert once, and documented the ReactiveSequence-last-child dependency. — `plan.md` Approach step 3
- [x] (must-fix) Consequences cited `marine_autonomy_integration_tests`/`done_hover` as local; they live in the separate `unh_marine_autonomy` repo — relabeled as a cross-repo manual check. — `plan.md` Consequences
- [x] (must-fix) `bt_task_navigator` has no test infra; moved the regression gtest to `marine_nav_behavior_tree` (existing harness), dropped the bootstrap. — `plan.md` Files to Change
- [ ] (confirmed) Phase 2 reasoning (re-read port insufficient; re-entry-on-id-change is the lever) verified against `hover_action.cpp:35-41` + `bt_action_node.hpp:237-250`.
