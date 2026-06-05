---
issue: 71
---

# Issue #71 — CrabbingPathFollower: progress-preserving localization + look-ahead steering + tunable/clamped yaw rate (part of #66)

## Local Review
**Status**: complete
**When**: 2026-06-05 01:51 -04:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**PR**: #72 at `7cd4879`
**Mode**: post-PR
**Depth**: Standard (reason: substantive Nav2 controller logic, single package)
**Specialists**: Static (cpplint/cppcheck), Claude Adversarial; Copilot present but not dispatched this pass
**Must-fix**: 1 fixed, 1 open | **Suggestions**: 2 addressed

### Findings
- [x] (must-fix) setPlan stall: a shorter/sparser same-goal re-plan clamped current_segment_ to segment_count (the "done" sentinel), parking the boat — fixed, clamp to segment_count-1 — `crabbing_path_follower.cpp` setPlan
- [ ] (must-fix, borderline) forward-only cursor can stick too far forward on a non-monotone same-goal path reshape (cursor never backs up); bounded one-segment backward correction proposed, pending design call — `crabbing_path_follower.cpp` computeVelocityCommands scan
- [x] (suggestion) look-ahead speed-scaled horizon uses commanded speed, not trajectory-derived per-pose speed — documented in-code
- [x] (suggestion) progress passed as start_offset can be negative; lookaheadPoint clamps to 0 (horizon measured from segment start) — documented in-code

### Static analysis
- cppcheck: clean on changed code (flagged items pre-existing: dt shadow, cos_error_azimuth scope)
- cpplint: header-guard for path_geometry.hpp fixed; remaining line-length/copyright findings match the package's existing un-enforced style (not new)

### Backward-compat (verified)
Defaults (lookahead 0, gain 1.0, max_yaw_rate π) reproduce the prior behaviour exactly; the localization change in setPlan is intentionally always-on (the teleport bug fix).
