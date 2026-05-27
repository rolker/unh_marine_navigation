---
issue: 28
---

# Issue #28 — Survey line-transition stall: uncommanded gap trips FCU 3s GUIDED watchdog

## Issue Review
**Status**: complete
**When**: 2026-05-27 13:02 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #28
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/28#issuecomment-4557000755
**Scope verdict**: well-scoped

### Actions
- [ ] Sequence after #35 (PR #36): let its "does FollowPath resend on a RUNNING path-input change?" spike resolve first; it answers #28's hot-swap feasibility. Re-measure the transition gap on #35's branch before committing to a separate #28 fix.
- [ ] In plan-task, decide the lever explicitly: (a) hot-swap path (no cancel/zero, depends on the spike) vs (b) bound the gap < 3 s (raise the transit-replan `RateController hz` + drop the pre-emptive `CancelAllNavigation`/zero); quantify the margin against the FCU 3 s watchdog.
- [ ] Land a command-continuity test with the fix (timing-dependent bug → deterministic repro on the `unh_echoboats_project11#186` dry-run/sim bed; assert no >Ns zero-command window across a line transition).
- [ ] Coordinate `run_tasks.xml` edits with #25 (PR #37) and verify no regression with #23 (stale per-line goal `header.stamp`); refresh the embedded `TreeNodesModel` if node usage changes.
- [ ] Honor "keep way, not zero" — note velocity_smoother/cmd_vel filters are disabled on BizzyBoat, so controller-output continuity is the whole fix (no smoothing layer to mask a gap).
