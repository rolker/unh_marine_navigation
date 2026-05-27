---
issue: 25
---

# Issue #25 — BT SkipUnknownTaskType catchall silently marks tracklines done on subtree FAILURE

## Issue Review
**Status**: complete
**When**: 2026-05-26 21:19 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #25
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/25#issuecomment-4550412892
**Scope verdict**: well-scoped

### Actions
- [ ] Surface the matched-but-failed reaction as a decision before implementing (DiagnosticStatus ERROR only vs. abort the run sequence) — operator-facing, confirm with Roland.
- [ ] Decide regression-test placement explicitly (this PR vs. #8); #18 already recurred once, so favor folding the test into this PR.
- [ ] Favor a stock BehaviorTree.CPP `Switch`/`IfThenElse` dispatch so the unknown-type exit and matched-but-failed exit are structurally distinct (prevents a #18-style recurrence at a new level; keeps ADR-0008 alignment).
