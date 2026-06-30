---
issue: 84
---

# Issue #84 — Expose crabbing-path-follower parameters through marine_control

## Issue Review
**Status**: complete
**When**: 2026-06-30 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #84
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The issue proposes adding a `marine_control::ControlServer` to
`marine_nav_crabbing_path_follower` so its tuning parameters are accessible from the
operator station via `rqt_marine_control` / `udp_bridge`. The change follows an
established pattern already present in `marine_nav_avoidance_controller` and
`marine_nav_ca_safety`. Change scope: `crabbing_path_follower.cpp`, its header,
`package.xml`, and `CMakeLists.txt` (the last is implicit but required — see below).

### Scope Assessment

- **Well-scoped?** Yes — single PR touching one package. Acceptance criteria are
  clear and testable.
- **Right repo?** Yes — `unh_marine_navigation` project repo under `core_ws`.
- **Dependencies?** `marine_control` package already exists at
  `layers/main/core_ws/src/marine_control/marine_control` and is used by sibling
  controllers. No blocking dependencies.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Change directly improves operator visibility and topside tunability of path-follower parameters |
| A change includes its consequences | Watch | Issue mentions `package.xml` but not `CMakeLists.txt` — implementer must also add `find_package(marine_control REQUIRED)` and the library in `target_link_libraries` (pattern: avoidance controller CMakeLists.txt) |
| Only what's needed | OK | Minimal extension; follows sibling pattern with no new abstractions |
| Test what breaks | Watch | Acceptance criteria say "tests pass" but add no new test for the marine_control integration path; consider a test that the ControlServer is constructed and parameters are bindable |
| Capture decisions, not just implementations | Watch | The dual-path update design (both `ros2 param set` and a marine_control `change` message write the same atomics) is load-bearing; code comment explaining this invariant will help future maintainers |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0008 — ROS 2 conventions | Yes | `package.xml` update must follow ROS 2 convention; `CMakeLists.txt` must use `find_package` + `ament_target_dependencies` / `target_link_libraries` pattern |
| ADR-0002 — Worktree isolation | N/A | Already in feature/issue-84 worktree |

### Consequences

- `CMakeLists.txt` also needs updating (`find_package(marine_control REQUIRED)` +
  link against `marine_control`) — the avoidance controller's CMakeLists.txt is the
  authoritative example. This is not explicitly listed in the acceptance criteria.
- If `deactivate()` must tear down `control_server_` (depends on whether
  `ControlServer` holds ROS subscriptions that should stop when the lifecycle
  node deactivates) — check avoidance controller's `deactivate()` for the pattern.

### Recommendations

- Cross-reference the `marine_nav_avoidance_controller` implementation precisely:
  the `activate()` hook (not `configure()`) is where `control_server_` is
  instantiated in that controller (line 311–312). Confirm whether `configure()` or
  `activate()` is the right hook for the crabbing follower, matching nav2 lifecycle
  semantics.
- Add at least a smoke-level gtest that instantiates the ControlServer path (similar
  to any existing param-callback tests).

### Actions
- [ ] Confirm `CMakeLists.txt` is updated with `marine_control` find/link (not listed in acceptance criteria but required to build).
- [ ] Confirm instantiation hook is `activate()` not `configure()` — check avoidance controller pattern vs. issue description wording.
- [ ] Consider adding a test for the marine_control binding path.
