---
issue: 82
---

# Issue #82 — Wire survey obstacle-avoidance controller (AvoidanceController) into marine_control

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-27 18:46 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (after addressing 1 must-fix)

**Branch**: feature/issue-82 at `6154a05`
**Mode**: pre-push
**Depth**: Standard (reason: safety-relevant nav2 plugin, new param surface, ~420 lines)
**Must-fix**: 1 (fixed) | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — must-fix fixed + regression-locked; suggestions are by-design/documented.

Two disjoint-lens adversarial passes. Both confirmed correct defaults parity,
declared-vs-bound symmetry, range-rejection over the channel, lifecycle/teardown,
topic/device uniqueness, and non-vacuous tests. Depends on marine_control PIC
(rolker/marine_control#11) to link into the plugin .so.

### Findings
- [x] (must-fix) `_range` declared DOUBLE_ARRAY → integer-literal bounds (`[1, 10]`) threw at declare before fallback, failing bring-up. Fixed: dynamic_typing + coerce int array; regression test added. — `avoidance_controller.cpp:declareAvoidanceControlParams`
- [ ] (suggestion, by-design) An operator can neutralize avoidance with in-range valid values (obstacle_avoidance_weight→0, max_deviation→min). The per-platform `_range` floors are the intended guard (a platform sets e.g. obstacle_avoidance_weight_range=[0.5,…] for a floor). Documented as operator authority; surfaced to Roland.
- [ ] (suggestion, documented) bind/reset + refreshParams() coherence rely on the controller_server being single-threaded (nav2 default); a multi-threaded compose would break bind-before-spin and could interleave a one-tick mixed param set. The param-read-per-tick pattern is pre-existing; the invariant is documented in an activate() comment.
- [ ] (deferred) package has pre-existing ament_lint debt on jazzy (cpplint header-guard/copyright, whole-file uncrustify mismatch, no copyright headers); untouched. New test is cpplint/uncrustify clean apart from the package's no-copyright-header convention.
