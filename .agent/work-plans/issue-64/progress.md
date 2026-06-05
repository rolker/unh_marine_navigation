---
issue: 64
---

# Issue #64 — Collision avoidance at survey speed: turn-before-slowdown is speed-limited (near-miss)

## Issue Review
**Status**: complete
**When**: 2026-06-04 19:23 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #64
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/64#issuecomment-4626804034
**Scope verdict**: needs-splitting

### Actions
- [ ] Split #64 into sub-issues (keep #64 as umbrella): (a) speed-scaled slowdown zones [config in seafloor/echoboats]; (b) custom e-stop node (yaw-preserving + reverse-arc) [this repo]; (c) yaw-sign-in-reverse sim verification spike [blocks (b)]; (d) recovery-BT back-off branch; (e) aft reflex cloud [if scope expands]
- [ ] Settle replace-vs-augment with the existing nav2 Collision Monitor (reconcile chain placement vs seafloor #25/#27/#36 + velocity_smoother) before planning the node
- [ ] Default reflex scope to forward-only + aft-gated reverse for June; defer all-around
- [ ] Treat as safety-critical: sim validation + regression tests in-scope, not follow-ups (yaw-sign-in-reverse, reverse closed-loop termination, speed-scaled zone extremes, escape-when-avoider-gave-up)
- [ ] ADR-0008: declare + validate node parameters (finiteness/domain), per #62 precedent
- [ ] Capture the safety-behavior decision (replace CM + autonomous reverse near obstacles) durably; operator-visible/configurable/abortable (CAMP annunciation)
- [ ] Coordinate speed-scaled polygon config with seafloor #3 (model split, in flight); principled fix tracked at seafloor #42
- [ ] Add /odom (mavros local_position/velocity_body) subscription; wire back-off into run_tasks.xml RecoveryNode

## Plan Authored
**Status**: complete
**When**: 2026-06-04 20:08 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-64/plan.md` at `5ebde97`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/68 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] Replace vs augment the nav2 Collision Monitor (plan assumes replace)
- [ ] Stale/absent pointcloud behavior (passthrough / hold-last / fail-safe stop)
- [ ] Confirm cmd_vel message type is TwistStamped against the live chain
- [ ] Capture the safety-behavior decision (PR rationale vs project docs/decisions)
