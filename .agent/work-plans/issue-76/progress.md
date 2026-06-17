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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-16 22:33 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-76 at `91bb181`
**Mode**: pre-push
**Depth**: Standard (reason: ~133 LOC across 5 files in a core_ws Nav2 controller plugin; safety-relevant boat code; work plan present)
**Must-fix**: 0 | **Suggestions**: 3

### Findings
- [ ] (suggestion) `gainScheduleScale` has no `isfinite` guard on `target_speed`; `std::max(NaN, v_min)` returns NaN, propagating to `crab_angle` when enabled. Cross-pass confirmed (Lens A + Lens B). Cannot occur in production — the only call site passes validator-guaranteed finite-positive `target_speed` — so it is pure-function-contract hardening, not a live defect. Optional: add `if (!std::isfinite(target_speed)) return crab_angle_deg;` + a test. — `path_geometry.hpp:130-138`
- [ ] (suggestion) No integration-level test that the param callback rejects `pid.gain_v_min <= 0` / non-finite sets; the "atomic can never hold 0" invariant is proven by construction but not at the callback boundary. Consistent with the package's existing tunables (shared gap, not a regression). — `crabbing_path_follower.cpp:248-256`
- [ ] (suggestion) Optional lock-in test: assert `gainScheduleScale(.., v_min, target_speed<0)` divides by `v_min` (negative-speed input is floored), documenting the contract edge. — `test_gain_schedule.cpp`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-17 06:24 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-76 at `a2fd6af`
**Mode**: pre-push
**Depth**: Standard (reason: ~134 LOC across 5 changed source files in a core_ws Nav2 controller plugin; safety-relevant boat code; work plan present)
**Must-fix**: 0 | **Suggestions**: 1

Re-review after hardening commit `a2fd6af`, which applied the 3 non-blocking
suggestions from the prior approved Local Review at `91bb181`: (1) `isfinite`
guard on `target_speed` in the pure `gainScheduleScale` (non-finite falls back
to `v_min`); (2) test lock-in for negative `target_speed` floored to `v_min`;
(3) test lock-in for non-finite `target_speed` yielding a finite result. Both
disjoint-lens Claude Adversarial passes (Lens A logic, Lens B systemic/safety)
returned "sound, no regression vs 91bb181". gtest 9/9 PASS (verified by running
the compiled `test_gain_schedule` binary). No lines exceed the 100-char ament
limit in the new code. Hardening commit confirmed correct and complete; the
isfinite guard is a strict superset — finite inputs traverse the identical path,
so all pre-hardening behavior is preserved. The new params still default to
disabled (`gain_ref_speed=0.0`); echoboats overlay activation is deliberately
deferred (not a gap). Known local uncrustify 0.78.1 drift excluded per scope.

### Findings
- [ ] (suggestion) `NonFiniteTargetSpeedStaysFinite` exercises NaN and +Inf but not -Inf; `std::isfinite` handles -Inf identically so this is a coverage-completeness note, not a correctness gap (finite-negative flooring is already covered by `FloorsNegativeTargetSpeedAtVMin`). — `test_gain_schedule.cpp:82-93`
