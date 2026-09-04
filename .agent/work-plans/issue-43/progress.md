---
issue: 43
---

# Issue #43 — Different-task mid-mission re-send must transit to the new task's start (#35 follow-up)

## Plan Authored
**Status**: complete
**When**: 2026-05-28 11:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-43/plan.md`
**PR**: TBD (draft on `feature/issue-43`)
**Phases**: single PR — two new C++ BT nodes + BT XML restructuring + tests + sim verification
**Scope**: ~3-5 hr focused work

### Open questions (carried from plan)
- [ ] Q1: `TaskIdUnchanged` as a C++ condition node (vs script-based comparison) — going with C++ since `TaskPtr` isn't directly scriptable. Confirm during implementation.
- [ ] Q2: `CaptureTask::halt()` should clear the captured slot (fresh entry re-latches). Confirm test behavior.
- [ ] Q3: Test fixture mirrors `test_dispatch_routing` (PR #37); don't add new mock-action-server infra.
- [ ] Q4: Confirm with #40 (PR #40) sim that the switch-detect path doesn't introduce a zero-cmd window.

### Sequencing
- **Must land after PR #36** — the reactive `SetPathFromTask` from #36 is what the switch-detect path needs to gate (same-task = continue reactive; different-task = halt to re-enter).
- Should ideally land **before next field deployment** so different-task re-sends transit correctly. PR #36's known-limitation callout documents the gap until #43 lands.
