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
