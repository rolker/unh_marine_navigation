---
issue: 87
---

# Issue #87 — Add turn-speed regulation to CrabbingPathFollower

## Issue Review
**Status**: complete
**When**: 2026-07-01 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #87
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Default off/neutral; live-settable via `marine_control`; finiteness-guarded. Issue is explicit on all three. |
| A change includes its consequences | Watch | Unit tests in acceptance criteria; plan phase should also confirm README/doc-comment coverage for new params. |
| Only what's needed | OK | Focused fix for a concrete, measured problem with field evidence. Single file + params + tests. |
| Improve incrementally | OK | Single PR scope; backward-compatible default. |
| Test what breaks | Watch | AC includes unit tests; key edge cases to require: crab_angle=0 (no-op), max angle (max regulation), NaN/Inf/negative inputs, default-off passthrough. Sim test in AC is broad — plan should concretize the scenario. |
| Safety First (project) | OK | Turn slowing is a net safety improvement; default neutral prevents regression. |
| Simulation-First Validation (project) | Watch | AC mentions sim test ("boustrophedon or zig-zag") but no specific sim package/scenario pinned — plan should resolve this. |
| Hardware Agnosticism (project) | OK | Regulation factor is speed-signal-based; no platform-specific path. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 — Worktree isolation | Yes | Already in worktree ✓ |
| 0008 — ROS 2 conventions | Yes | Param declaration, callback, and range descriptor must follow existing patterns (`gainScheduleScale`, `setSpeedLimit` are cited precedents). |
| 0001 — Adopt ADRs | Recommendation | The choice between curvature-based vs. crab-angle-based regulation is a non-obvious design decision. Record the rationale (ADR or detailed plan comment) so future contributors don't accidentally revert or misapply the approach. |

### Consequences

- New params live in `marine_nav_crabbing_path_follower`; must also be bound in the `marine_control` server (same pattern as `default_speed`, `gain_ref_speed`) — verify no marine_control-side changes are needed beyond what `setSpeedLimit`/`declare_parameter` already provides.
- If curvature-based approach is chosen, path curvature computation must handle degenerate inputs: path with fewer than 2 lookahead points, collinear points (infinite radius), NaN waypoints.
- Speed precedence chain (#32) must stay coherent — regulation is a cap applied after the precedence winner, not a new speed source. The plan must show where in the precedence chain the regulation step sits (the issue spec says "after precedence winner, mirroring `speed_limit_`").

### Actions
- [ ] Pin a concrete sim scenario in the plan (package, launch, expected behavior) so the sim acceptance criterion is verifiable, not aspirational.
- [ ] Decide and record (plan or ADR note) which regulation signal is used (curvature vs. crab-angle magnitude) and why — the "evaluate both" framing in the issue is exploratory; the plan phase should close on one approach.
- [ ] Confirm that marine_control integration requires no additional binding changes (or add them to scope if it does).
- [ ] Ensure unit tests cover degenerate inputs: NaN/Inf/negative regulation factor, zero crab_angle (no-op path), default-params passthrough.
