---
issue: 91
---

# Issue #91 — Fix look-ahead double-correction in CrabbingPathFollower

## Issue Review
**Status**: complete
**When**: 2026-07-01 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #91
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope
Single bug fix in `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp`
(lines 930–932): replace the boat-to-look-ahead-point bearing with the segment
azimuth at the look-ahead point. The fix is confined to one controller and one
logical change. Acceptance criteria include a unit test and a sim/log check — both
are clearly bounded. Single PR.

**Right repo?** Yes — `unh_marine_navigation` project repo, `marine_nav_crabbing_path_follower` package.

**Dependencies**: #90 (merged) is a logical predecessor (curvature anticipation is
gated on stable look-ahead). No blocking dependency on open issues.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix is clearly motivated; existing DEBUG log at line 897 continues to expose `base_heading` and `crab_angle`. Note: that log fires *before* the look-ahead block so after the fix `base_heading` in the log will reflect the updated look-ahead azimuth — confirm it still makes sense. |
| A change includes its consequences | Watch | Issue specifies a unit test for `base_heading` derivation and a sim/log check. The implementation will need a new pure function in `path_geometry.hpp` (e.g. `lookaheadSegmentAzimuth`) to retrieve the azimuth of the segment the look-ahead point lands on — `lookaheadPoint()` returns only a `Point`, not the segment index. The new function must be unit-tested alongside it. |
| Test what breaks | OK | Acceptance criteria explicitly require a unit test for the `base_heading` derivation and a monotonic cross-track error sim check. Existing test patterns in `test_path_geometry.cpp` are directly reusable. |
| Improve incrementally | OK | Narrow, targeted fix. Does not touch the crab PID, speed regulation, or curvature logic. |
| Only what's needed | OK | Fix is minimal. One new helper function in `path_geometry.hpp`, one changed call site in `crabbing_path_follower.cpp`. |
| Capture decisions, not just implementations | OK | The issue body itself is the decision record (root cause, rejected alternative, rationale). No ADR needed for a bug fix. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0008 — ROS 2 conventions | Yes | Fix touches a Nav2 controller plugin; follow ROS 2 Jazzy/Rolling conventions. No deviation expected. |
| ADR-0013 — progress.md vocabulary | Yes | This entry uses `## Issue Review` per ADR-0013. |
| Others | No | No workspace infra, Python packaging, build system, or agent-instruction changes. |

### Consequences

- `path_geometry.hpp` will likely gain a new function (`lookaheadSegmentAzimuth` or
  a combined `lookaheadPointAndSegment`). Corresponding tests should be added to
  `test_path_geometry.cpp`.
- The DEBUG log at line 897 logs `segment_azimuth` before the look-ahead block.
  After the fix the logged value will be the look-ahead segment azimuth (correct),
  but the label in the log message should be verified to still be accurate.
- No message-type or interface changes → no downstream package updates needed.

### Recommendations

- Define a pure `lookaheadSegmentAzimuth(poses, start_seg, start_offset, lookahead)`
  (or equivalent) in `path_geometry.hpp` rather than inlining the segment-walk
  logic a second time in `crabbing_path_follower.cpp`. Keeps the geometry layer
  testable in isolation, consistent with the existing `lookaheadPoint` pattern.
- Update the DEBUG log string at line 897 if needed so the label matches the
  semantics of the value being logged (look-ahead segment azimuth vs. current
  segment azimuth).

### Actions
- [ ] Add `lookaheadSegmentAzimuth` (or combined `lookaheadPointAndSegment`) to `path_geometry.hpp` and unit-test it in `test_path_geometry.cpp`.
- [ ] Verify/update DEBUG log string at line 897 after changing the `base_heading` computation.

## Plan Authored
**Status**: complete
**When**: 2026-07-01 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-91/plan.md` at `3c60486`
**Branch**: feature/issue-91 at `3c60486`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-01 11:53 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-91/plan.md` at `3c60486`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) `lookaheadSegmentAzimuth` should return the final segment's azimuth when `start_seg` is past the last segment (mirror `lookaheadPoint`'s goal-clamp, `path_geometry.hpp:82-84`), not fold it into the `0.0`-degenerate bucket — add a parallel out-of-range test. — `plan.md:47`
- [ ] (suggestion) Note in the updated comment block that the segment-tangent `base_heading` steps discretely at vertex crossings (bounded by the `max_yaw_rate` clamp) — `plan.md:38`
- [ ] (suggestion) Confirm the sim/log monotonic cross-track decay acceptance check is run manually at review-code (it's not an automated deliverable). — `plan.md:66`
- [ ] (info) Plan correctly determined the DEBUG log at `crabbing_path_follower.cpp:897` needs no change — it logs `segment_azimuth` (current segment, line 815), not `base_heading`; the review-issue log action item is discharged.
