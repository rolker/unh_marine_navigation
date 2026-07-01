# Plan: Fix look-ahead double-correction in CrabbingPathFollower

## Issue

https://github.com/rolker/unh_marine_navigation/issues/91

## Context

With look-ahead enabled (`lookahead_distance > 0` or `lookahead_time > 0`),
`CrabbingPathFollower` sets `base_heading` to the bearing from the boat's current
position to the look-ahead point (pure-pursuit steering). The crab-angle PID then
adds a cross-track correction on top, resulting in double correction: the
pure-pursuit bearing *already* steers back toward the line, and the crab PID drives
the same error a second time. This causes oscillation that is worst at short horizons
(≈1–2 s at survey speed, where the bearing swings ≈50° per metre of cross-track
error).

The fix: when look-ahead is on, set `base_heading` to the **path tangent at the
look-ahead point** (the azimuth of the segment the look-ahead point falls on), not
the boat-to-point bearing. This anticipates upcoming bends while leaving cross-track
correction solely to the crab PID — no double-counting.

## Approach

1. **Add `lookaheadSegmentAzimuth` to `path_geometry.hpp`** — a pure inline function
   that walks the path with the same logic as `lookaheadPoint` but returns the
   azimuth (radians) of the segment containing the look-ahead point. Degenerate
   inputs (empty path, single point, zero-length segment) return `0.0`. Sits
   alongside `lookaheadPoint` and follows the same "pure, no ROS scaffolding"
   contract so it is unit-testable without a node.

2. **Replace the `atan2` call in `crabbing_path_follower.cpp` (lines 930–932)** —
   swap `base_heading = AngleRadians(atan2(la_point.y − boat.y, la_point.x − boat.x))`
   for `base_heading = AngleRadians(lookaheadSegmentAzimuth(...))`. `la_point` is
   still computed by the preceding `lookaheadPoint` call (the curvature block at
   lines 979–1007 reuses it as the third circumfit point — do not remove it).

3. **Update the comment block at lines 899–905** — correct "pure-pursuit bearing to a
   point" to "path tangent at the look-ahead point" to match the fixed semantics.

4. **Add unit tests in `test_path_geometry.cpp`** — four tests for
   `lookaheadSegmentAzimuth`:
   - Straight line: returns the segment azimuth regardless of cross-track offset
     (the acceptance-criteria unit test — boat-position-independence).
   - Bend anticipation: look-ahead past a turn returns the next segment's azimuth,
     not the current one.
   - Past-end clamp: returns the final segment's azimuth when the horizon overshoots.
   - Degenerate inputs: empty path and single-point path do not crash.

## Files to Change

| File | Change |
|------|--------|
| `include/marine_nav_crabbing_path_follower/path_geometry.hpp` | Add `lookaheadSegmentAzimuth(poses, start_seg, start_offset, lookahead) → double` |
| `src/crabbing_path_follower.cpp` | Replace lines 930–932 (`atan2` bearing) with `lookaheadSegmentAzimuth` call; update comment at 899–905 |
| `test/test_path_geometry.cpp` | Add four `LookaheadSegmentAzimuth` tests using the existing `makePath` helper |

No CMakeLists changes needed — `test_path_geometry.cpp` is already registered as
`test_path_geometry` in the package's `CMakeLists.txt` (line 71).

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Unit tests for new function; comment update; `la_point` computation preserved for curvature block |
| Test what breaks | Four targeted unit tests; acceptance criteria include a sim/log check (cross-track decays monotonically with look-ahead on) |
| Only what's needed | One new function (~20 lines), one changed call site, four tests; no interface changes, no param changes |
| Human control and transparency | Existing DEBUG log at line 897 logs `segment_azimuth` (current segment, unchanged); label remains accurate |
| Improve incrementally | Narrow fix; does not touch crab PID, speed regulation, or curvature logic |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 — ROS 2 conventions | Yes | Fix stays within the Nav2 controller plugin interface; no convention deviation |
| ADR-0013 — progress.md vocabulary | Yes | Progress file uses `## Plan Authored` entry per ADR-0013 |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `path_geometry.hpp` (new function) | `test_path_geometry.cpp` (unit tests) | Yes |
| `base_heading` computation | Comment at lines 899–905 | Yes |
| `base_heading` semantics | DEBUG log label (line 897 logs `segment_azimuth`, not `base_heading` — label stays correct) | Yes — verified, no change needed |

## Open Questions

- None — approach, affected files, and acceptance criteria are fully specified by the issue.

## Estimated Scope

Single PR.
