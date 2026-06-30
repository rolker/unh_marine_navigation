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

## Plan Authored
**Status**: complete
**When**: 2026-06-30 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-84/plan.md` at `2e799fe`
**Branch**: feature/issue-84 at `2e799fe`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-30 15:13 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-84/plan.md` at `2e799fe`
**PR**: PR-less (--issue mode; local feature/issue-84)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Wrapped-by-avoidance topic collision: `marine_nav_avoidance_controller` configures its inner controller under the **same** name (`avoidance_controller.cpp:271`) and calls `primary_->activate()` (`:291`). Once the crabbing follower also creates a `ControlServer` in `activate()` on `~/control/<plugin_name_>/state|change`, a wrapped crabbing follower and its wrapper both advertise on identical topics → two colliding state publishers exposing different control sets; the panel sees an incoherent set. Standalone crabbing is fine. The plan's "per-plugin topics keep multiple controllers from colliding" assumes distinct names; the wrap case shares one. Decide before implementing: scope to standalone + document, differentiate the inner's topic/name when wrapped, or have the wrapper suppress/forward the inner's server. — `plan.md:4` (step 4) / Consequences table `plan.md:92-97`
- [ ] (must-fix) `default_speed` won't receive a `FloatingPointRange` from `declareCrabbingControlParams()`: it is already declared at `crabbing_path_follower.cpp:43`, before any "before the read_validated block" insertion point, so the helper's `declare_parameter_if_not_declared` no-ops for it (the descriptor only attaches at first declaration). Moreover, attaching a range descriptor to `default_speed` changes declare-time validation and can preempt the existing graceful invalid-value fallback (`cpp:52-107`) — an out-of-range YAML value would now throw at declare instead of falling back. The plan treats all params uniformly; `default_speed` needs explicit handling (attach the descriptor at its existing declaration site with a range permissive enough to preserve the fallback, or exclude it from the generic helper and bind it separately). — `plan.md:21-37`
- [ ] (must-fix) Missing test build dependencies: the new `test_crabbing_control.cpp` mirrors `test_avoidance_control.cpp`, which `#include`s `marine_control_interfaces/msg/*`. The sibling declares `<test_depend>marine_control_interfaces</test_depend>` (package.xml) and `ament_target_dependencies(test_avoidance_control marine_control marine_control_interfaces rclcpp rclcpp_lifecycle)` + `target_link_libraries(... ${PROJECT_NAME})`. The plan's step 8 adds only `<depend>marine_control</depend>` and step 9 omits `marine_control_interfaces` and the test-target link — the test won't build as written. — `plan.md:54-57`, `plan.md:71-73`
- [ ] (suggestion) Param count mismatch: step 1 says "Params to expose (9)" but then lists 10 (`default_speed` + 9 others). Reconcile — likely tied to the `default_speed` handling decision above (9 simple tunables via the helper, `default_speed` handled separately). — `plan.md:27`
- [ ] (suggestion) Confirm the ADR reference: "ADR-0003 (marine_control D4–D6)" is the external marine_control device-control ADR (cf. `test_avoidance_control.cpp:2` "unh_marine_autonomy#140 / ADR-0003"), not this workspace's ADR-0003 (`workspace-infrastructure-is-project-agnostic`). The plan complies by mirroring the avoidance pattern exactly; just disambiguate the citation. — `plan.md:90`
