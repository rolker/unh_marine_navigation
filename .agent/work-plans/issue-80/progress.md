---
issue: 80
---

# Issue #80 — Wire collision-avoidance node (CaSafetyNode) into marine_control device control

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-27 17:27 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-80 at `42051a0`
**Mode**: pre-push
**Depth**: Standard (reason: safety-critical CA brake node + ~295 lines)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; both adversarial lenses confirmed correct threading, validation routing, construction order, teardown.

Two disjoint-lens Claude adversarial passes cross-confirmed: all 15 live params
bound (no typos), the change channel routes through the existing onSetParameters
validation (finite/>0 + slowdown_min<=max ordering) so no atomic is
exposed-but-unvalidated, single-threaded executor makes the ControlServer
callbacks non-concurrent with node callbacks, construction-last + teardown order
satisfy the control_server.hpp contract, and the launch tests are non-vacuous.

### Findings
- [ ] (suggestion) 13 dynamic doubles have no FloatingPointRange → panel sees degenerate [0,0] bounds AND an operator can shrink stop_length/widths toward ~0 (valid but weakens the brake). Adding per-param sane ranges needs Roland's domain values — DEFERRED as a follow-up decision. — `ca_safety_node.h:declareDynamicDouble`
- [ ] (suggestion) safety-critical adopter (ADR-0003 D8.3) uses bare fire-and-forget + audit log, no operator-confirmation handshake — by design; tracks with the marine_control confirmation follow-up. — `ca_safety_node.h:setupControlServer`
- [ ] (deferred) package has pre-existing ament_lint debt on jazzy (cpplint header-guards/copyright, whole-file uncrustify config mismatch, no copyright headers) — out of scope; the new launch test is flake8/pep257 clean and matches the no-header convention.
