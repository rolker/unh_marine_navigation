---
issue: 76
---

# Issue #76 — marine_nav_crabbing_path_follower: speed-normalize the cross-track gain (gain schedule)

## Issue Review
**Status**: complete
**When**: 2026-06-16 18:30 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Issue**: #76
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/76#issuecomment-4725111013
**Scope verdict**: well-scoped

### Actions
- [ ] Add unit test for gain-schedule scaling (test that crab_angle scales correctly at multiple speeds, with gain_ref_speed disabled vs enabled, and with the gain_v_min floor) — test what breaks principle
- [ ] Add in-source rationale comment at the scaling insertion point explaining why target_speed (commanded) is used rather than measured speed, and the gain_v_min floor purpose
- [ ] Update platform config (`unh_echoboats_project11` nav2 overlay) with `pid.gain_ref_speed: 1.8` and `pid.gain_v_min: 0.5` — note whether this is part of the same PR or a companion commit
- [ ] Update parameter documentation if a parameter reference doc exists for marine_nav_crabbing_path_follower

## Plan Authored
**Status**: complete
**When**: 2026-06-16 19:05 +00:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-76/plan.md` at `d7b59fc`
**Branch**: feature/issue-76 at `d7b59fc`
**Phases**: single

The plan speed-normalizes the cross-track gain by scaling the PID output
`crab_angle` by `gain_ref_speed / max(target_speed, gain_v_min)` after the PID
compute (L528). Two new live-tunable atomics (`pid.gain_ref_speed` default
**0.0 = disabled**, `pid.gain_v_min` default **0.5**) follow the existing
`lookahead_*` pattern (atomic + `read_validated` + SetParameters branch). The
scaling math is extracted to a pure `gainScheduleScale()` in `path_geometry.hpp`
and unit-tested in a new `test/test_gain_schedule.cpp` (disabled, multi-speed,
`v_min` floor, sign). In-source rationale comment at the insertion point covers
why commanded (not measured) speed and the floor purpose. No README/param-ref
doc exists for this package — in-source comments are the doc.

**Scope note**: `unh_echoboats_project11` nav2-overlay activation (`gain_ref_speed:
1.8` / `gain_v_min: 0.5`) is **out of scope / deferred** to a separate follow-on
issue — the 1.8 anchor needs a sim + on-water re-test first (issue Caveat). This
PR lands in `unh_marine_navigation` only.

### Open questions
- [ ] No blocking open questions. Optional reviewer flag: `gain_v_min` uses `> 0`
      (guarantees a non-zero divisor) rather than `>= 0` — confirm at review-plan.
- [ ] Action-item carry-forward: the echoboats overlay activation + `pid.p −13`
      reconciliation is the deferred follow-on issue (not this PR).

## Plan Review
**Status**: complete
**When**: 2026-06-16 20:30 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Plan**: `.agent/work-plans/issue-76/plan.md` at `d7b59fc`
**PR**: PR-less (no draft PR at review time)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) `gain_v_min > 0` is the correct choice — divisor can never reach zero, confirmed. The plan's open-question flag (`>= 0` alternative) is resolved: keep `> 0`. — `plan.md:51`
- [ ] (suggestion) SetParameters param name namespace: the plan names the params `pid.gain_ref_speed` / `pid.gain_v_min`. The existing `SetParameters` callback uses `base = plugin_name_` and branches on `base + ".lookahead_distance"` etc. The new branches will need to match `base + ".pid.gain_ref_speed"` (not `base + ".gain_ref_speed"`), consistent with `plugin_name_ + ".pid.reset_threshold_seconds"` at line 39. The plan's `read_validated` calls and `declare_parameter_if_not_declared` declarations must also use the `.pid.` sub-namespace. This is consistent with the plan's stated naming but worth making explicit so the implementer doesn't accidentally drop the `.pid.` prefix. — `plan.md:48,54`
- [ ] (suggestion) All four review-issue action items are addressed by the plan: unit test (step 7), in-source rationale (steps 2+6), platform config deferred per operator decision (Consequences table), parameter doc noted as in-source-only (step 8). The platform-config action item from issue-review is legitimately deferred; no gap remains in this PR's scope.
