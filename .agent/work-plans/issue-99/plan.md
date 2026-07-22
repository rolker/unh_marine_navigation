# Plan: CrabbingPathFollower — geometry-preserving segment cursor across same-goal setPlan re-issues

## Issue

https://github.com/rolker/unh_marine_navigation/issues/99

## Context

`CrabbingPathFollower::setPlan` (crabbing_path_follower.cpp:684) preserves `current_segment_`
**by raw index** across same-goal re-issues, with `min(index, segment_count−1)` as the
shorter-path fallback. Under the AvoidanceController the inner follower alternates between the
sparse N-waypoint nominal path and dense 2 m station-resampled reshapes of the same goal, so a
dense-path index caps onto the **final leg** of the sparse path. Bag-proven on 2026-07-21
(Massabesic, echoboats#381): the boat abandoned an 8-waypoint return route at leg 1 and sailed
away steering against leg 7. The index-preservation exists to keep the 2026-06-04 anti-snap-back
property (#250: re-localizing from path start during a weave kicked the PID); the fix must keep
that property while making the preserved quantity geometric, not an index.

## Approach

1. **Extract a pure cursor-mapping helper** into `path_geometry.hpp` (matches the existing
   inline-helper + gtest pattern):
   `mapCursorToNewPath(old_poses, old_segment_index, new_poses) -> new_segment_index`.
   Anchor = the old current segment's **start point**. Hybrid mapping (per plan review):
   restrict candidates to a **bounded arc-length window** around the old cursor's arc-length
   position (window = max(25 m, 10% of new path length)), pick the nearest-point segment within
   the window; fall back to global nearest with arc-fraction tie-break only if the window yields
   no sane candidate (log when that happens). The window is what prevents capture by an
   **adjacent parallel leg** on boustrophedon patterns when a lateral offset approaches half the
   leg spacing — adjacent legs are far away in arc length even when close in space. Geometric
   anchoring (not raw arc length alone) survives density flips **and** truncated re-issues
   (BT-pruned transit paths whose start advances). Fast path: identical pose count + endpoints
   → keep the cursor unchanged. Guard degenerate inputs (empty / single-pose new path → 0),
   matching the defensive style of the other `path_geometry.hpp` helpers.
2. **Use it in `setPlan`**: keep the `new_line` goal-moved reset exactly as-is; in the same-goal
   branch replace index carry-over + cap with the mapping helper. Clamp the result to
   `[0, segment_count−1]` — never the `segment_count` done-sentinel, so a shortened re-issue
   near the goal finishes via the normal forward scan + goal checker instead of stalling.
   Rewrite the 06-04 comment block to document the geometric invariant, including the
   interaction with the compute-side forward scan: the scan only advances, so the mapper may
   legitimately land the cursor at a segment whose start is slightly behind the boat's
   projection — the scan re-advances within the same cycle; the mapper must never place the
   cursor far backward (that is the #250 snap-back the window bound prevents).
3. **Delete the `min(index, segment_count−1)` cap branch** (subsumed by the helper's clamp);
   keep a debug log when the mapped index differs from the old index by > 1 segment.
4. **Unit tests** in a new `test/test_plan_cursor.cpp` (pure helper, no ROS node needed):
   - 07-21 regression: 8-waypoint zigzag; cursor on leg 1 of the dense (2 m-resampled) variant
     maps back to leg 1 of the sparse variant — never leg 7 — and round-trips sparse→dense.
   - Laterally offset (≤ 6 m corridor-style) same-goal variant maps to the same leg.
   - Truncated same-goal re-issue (drop the first k poses): cursor maps to the geometrically
     same location, not index-shifted.
   - Shortened path with old cursor past the new end: clamps to last traversable segment,
     never `segment_count`.
   - Adversarial parallel-leg case: boustrophedon legs 8 m apart, current leg reshaped 6 m
     toward the neighbor (anchor strictly nearer the wrong leg) — window mapping must stay on
     the current leg where global-nearest would capture the neighbor.
   - Reference-step bound (purely geometric — must NOT rely on the #66 slew limiter, which
     defaults to 0.0/off): across a dense↔sparse flip mid-leg with an on-path pose, the
     cross-track error to the mapped segment changes by < 0.5 m.
   - Degenerate inputs: empty and single-pose new paths → cursor 0, no UB.
   - Anti-snap-back (#250 property): mapping is pose-independent and monotone under forward
     progress — repeated same-shape re-issues never move the cursor backward.
5. **Build + test** via `./core_ws/build.sh marine_nav_crabbing_path_follower` and
   `./core_ws/test.sh marine_nav_crabbing_path_follower`; run `/review-code` pre-push.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_crabbing_path_follower/include/.../path_geometry.hpp` | Add `mapCursorToNewPath` (+ small arc-length helpers if needed) |
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` | `setPlan`: same-goal branch uses the helper; remove cap branch; rewrite comment |
| `marine_nav_crabbing_path_follower/test/test_plan_cursor.cpp` | New gtest suite (cases in step 4) |
| `marine_nav_crabbing_path_follower/CMakeLists.txt` | Register the new test |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | Every failure mode from the field event becomes a named test case; the fix is landed with its regression suite |
| A change includes its consequences | 06-04 comment block rewritten; cap branch removed rather than left dead; deploy consequence (gabby core_ws rebuild) recorded in the PR |
| Only what's needed | AvoidanceController always-resample companion and large-cross-track acquisition mode deliberately out of scope (separate issues) |
| Improve incrementally | Pure-helper extraction reuses the existing path_geometry test pattern instead of introducing a node-level test harness |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 (ROS 2 conventions) | Yes | nav2_core::Controller contract unchanged; no API/param changes |
| 0018 (local-first CI) | Yes | Merge may use full-scope `ci_local.sh` attestation on the PR head |
| Others | No | Workflow-level only |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| setPlan cursor semantics | 06-04 rationale comment in setPlan | Yes (step 2) |
| Cursor behavior shared by BEN/Izzy/seafloor echoboats | Sim/straight-line sanity via existing test suite + echoboat240 sim scenario | Partially — sim zigzag scenario tracked in echoboats#381 thread 2b, not this PR |
| Fix effective on Bizzy | gabby core_ws rebuild before next deployment | No — deployment step, noted in PR body |

## Open Questions

- ~~Tie-break strategy~~ — resolved per plan review: bounded arc-length window primary,
  global nearest + arc-fraction tie-break as logged fallback.
- Land the AvoidanceController always-resample companion as a separate issue now, or wait until
  this fix soaks in sim?

## Estimated Scope

Single PR (helper + setPlan change + tests). Sim-scenario validation shared with nav#100 tracked
under echoboats#381.
