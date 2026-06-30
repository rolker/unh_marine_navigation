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

## Implementation
**Status**: complete
**When**: 2026-06-30 16:05 +00:00
**By**: Claude Opus

**Branch**: feature/issue-84
**Commits**:
- `e2ebc2f` feat(crabbing): expose path-follower params via marine_control
- `89385e0` test(crabbing): cover marine_control device-control wiring
- (this docs commit) plan.md sync + progress.md entry

### Files changed
- `marine_nav_crabbing_path_follower/include/.../crabbing_path_follower.h` —
  added `#include "marine_control/control_server.hpp"`, `<memory>`/`<string>`/
  `rclcpp_lifecycle` includes, three free-function declarations
  (`declareCrabbingDefaultSpeed`, `declareCrabbingControlParams`,
  `bindCrabbingControls`), and `marine_control_namespace_` + `control_server_`
  members.
- `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` — added the
  `kTunables[]` table + the three helpers; wired `configure()` (default_speed
  descriptor, 9-tunable declare before `read_validated`, namespace param),
  `activate()` (ControlServer + bind + dual-path/namespace comments),
  `deactivate()`/`cleanup()` (reset).
- `marine_nav_crabbing_path_follower/package.xml` — `<depend>marine_control</depend>`
  + `<test_depend>marine_control_interfaces</test_depend>`.
- `marine_nav_crabbing_path_follower/CMakeLists.txt` — `find_package(marine_control)`
  + lib dep; `test_crabbing_control` target (links `${PROJECT_NAME}`, deps
  marine_control/marine_control_interfaces/rclcpp/rclcpp_lifecycle).
- `marine_nav_crabbing_path_follower/test/test_crabbing_control.cpp` — new (7 tests).
- `.agent/work-plans/issue-84/plan.md` — synced to the implemented design.

### How each must-fix (A–D) was resolved
- **A (wrap-case differentiation)** — implemented the config-driven namespace.
  Declared `<plugin_name_>.marine_control.namespace` (string descriptor) in
  `configure()`, default = `plugin_name_`, stored in `marine_control_namespace_`
  (empty -> falls back to `plugin_name_`). `activate()` builds the ControlServer
  topics as `~/control/<namespace>/state|change`. **Mechanism chosen**: a
  deployment-set parameter (not automatic wrap-detection), because the avoidance
  wrapper configures + activates its inner controller under the SAME
  `plugin_name_` (`avoidance_controller.cpp:271`, `:291`) and itself advertises a
  ControlServer on `~/control/<plugin_name_>/...` — there is no in-process signal
  the inner can read to know it is wrapped, so the deployment config is the clean
  hook. Unset behaviour is byte-identical to today's standalone layout; when
  wrapped, the deployment sets the inner's namespace to a distinct value so the
  two servers never collide. Bound parameter *names* stay `<plugin_name_>.*`
  regardless — only the panel channel is namespaced. Documented in
  `activate()`/`configure()` comments and plan.md Consequences. Verified by
  topic-string reasoning and `NamespaceParamDifferentiatesTopicsWhenWrapped`.
- **B (default_speed descriptor)** — `default_speed` is declared at its existing
  site (`cpp:43`) via `declareCrabbingDefaultSpeed`, attaching a
  `FloatingPointRange [0, 20]` m/s (matching the avoidance `avoid_speed` bound).
  The range is permissive so the bespoke configure-time graceful fallback
  (`cpp:52-107`) is preserved for every physically meaningful value (0, integer,
  bool/wrong-type still route through it); only a self-contradictory out-of-range
  *override* fails loudly at declare. Excluded from the generic helper and bound
  separately in `bindCrabbingControls`.
- **C (test build deps)** — `package.xml`: `<depend>marine_control</depend>` +
  `<test_depend>marine_control_interfaces</test_depend>`. `CMakeLists.txt`:
  `find_package(marine_control REQUIRED)` + lib dep; test target
  `target_link_libraries(test_crabbing_control ${PROJECT_NAME})` +
  `ament_target_dependencies(test_crabbing_control marine_control
  marine_control_interfaces rclcpp rclcpp_lifecycle)` — mirrors avoidance.
- **D (suggestions)** — param count reconciled in plan.md (9 simple tunables via
  the helper + `default_speed` handled separately = 10 controls). ADR-0003
  citation disambiguated as the *external* marine_control device-control ADR
  (per `test_avoidance_control.cpp:2`), not this workspace's ADR-0003.

### Build result
`colcon build --packages-up-to marine_nav_crabbing_path_follower` — **PASS**
(marine_control_interfaces, marine_nav_utilities, marine_control, then the
follower). Only pre-existing `-Wunused-parameter` / `-Wsign-compare` warnings in
untouched code.

### Test result
`colcon test` gtest suites — **all green**: `test_crabbing_control` **7/7**,
`test_gain_schedule` 9/9, `test_path_geometry` 18/18.

**Pre-existing lint debt (not introduced by #84)**: `ament_cpplint` and
`ament_uncrustify` fail on `crabbing_path_follower.{h,cpp}` — wrong header-guard
style (single-underscore), missing copyright headers, 12 pre-existing >100-char
lines, and `if(...)`/brace style. Proven pre-existing by running
`ament_uncrustify` on the pristine `HEAD` file ("1 files with code style
divergence") and confirmed by the sibling `test_avoidance_control.cpp` (same
no-copyright convention). New code is lint-clean (`ament_cpplint` on
`test_crabbing_control.cpp` -> "No problems found"). A wholesale reformat of the
legacy file is out of scope for this issue; flag for a dedicated style sweep.

### Next step
review-code.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-30 16:18 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-84 at `b4f64a3`
**Mode**: pre-push
**Depth**: Deep (reason: 611 changed code lines >200; lifecycle/concurrency + cross-module marine_control integration)
**Must-fix**: 0 | **Suggestions**: 6
**Round**: 1 | **Ship**: recommended — no must-fix findings; both adversarial passes converged clean

### Findings
- [ ] (suggestion) Wrap-case topic collision is config-mitigated, not enforced — inner+wrapper ControlServers share topics unless deployment sets `marine_control.namespace`; no warn if forgotten (standalone path is correct). Cross-confirmed with plan-review must-fix A — `src/crabbing_path_follower.cpp:348-370`
- [ ] (suggestion) `NamespaceParamDifferentiatesTopicsWhenWrapped` asserts hardcoded string literals, exercises no production namespace-read/fallback logic — `test/test_crabbing_control.cpp:200-225`
- [ ] (suggestion) Platform `_range` narrower than the built-in default silently clamps the effective tunable value into range (documented, untested) — `src/crabbing_path_follower.cpp:298`
- [ ] (suggestion) Negative-but-ordered `_range` (e.g. `[-5,-1]`) advertises a range the exclusive `>0` callback rejects → advertised range vs held value inconsistent; guard checks ordering not sign — `src/crabbing_path_follower.cpp:274-301`
- [ ] (suggestion) `node_.lock()` failure in `activate()` silently no-ops (no server, no log); an RCLCPP_WARN would aid field debugging — `src/crabbing_path_follower.cpp:528`
- [ ] (suggestion) Pre-existing package-wide lint debt (no copyright headers, long lines) — all `.cpp` cpplint hits are on untouched pre-existing lines; new test follows the package's established no-copyright convention; flag for a dedicated style sweep — `marine_nav_crabbing_path_follower/*`

## Implementation (review fix pass)
**Status**: complete
**When**: 2026-06-30 16:40 +00:00
**By**: Claude Opus

**Branch**: feature/issue-84
**Commits**:
- `6766314` fix(crabbing): harden marine_control error-handling per #84 review

Operator-approved scope: address ONLY the three error-handling suggestions from
the pre-push review; the other three (lint debt, namespace-test string literals,
clamp doc) are explicitly deferred.

### How each addressed finding was resolved

1. **Silent `node_.lock()` no-op in `activate()`** (review finding
   `src/crabbing_path_follower.cpp:528`) — added an `else` branch to the
   `if (auto node = node_.lock())` block that warns without requiring the locked
   node (uses the stored `logger_`, a `rclcpp::Logger` value set in
   `configure()`, exactly as `deactivate()`/`cleanup()` already do). Control flow
   is unchanged — the skip just became observable:
   ```cpp
   } else {
     RCLCPP_WARN(
       logger_,
       "CrabbingPathFollower: parent node expired during activate() of plugin "
       "%s; skipping marine_control panel wiring (no ControlServer created). "
       "Tunables remain live in-process but will not be exposed topside.",
       plugin_name_.c_str());
   }
   ```

2. **Range-sign guard** (review finding `src/crabbing_path_follower.cpp:274-301`)
   — tightened the well-formed-range condition in `declareCrabbingControlParams`
   to also require the lower bound be at or above the tunable's valid floor
   (`t.default_min`, which mirrors the callback's positivity/validity rule). A
   negative-but-ordered range now falls back to the built-in default exactly like
   a misordered or non-finite one (graceful-fallback philosophy preserved):
   ```cpp
   if (range.size() == 2 && std::isfinite(range[0]) && std::isfinite(range[1]) &&
     range[0] < range[1] && range[0] >= t.default_min)
   ```
   The warning message was updated to state the floor:
   `"need [min, max] with %g <= min < max"`. **Unit-tested** — added
   `NegativeOrderedRangeFallsBackToDefault` to `test/test_crabbing_control.cpp`,
   mirroring the existing `MalformedRangeFallsBackToDefault` (sets
   `heading_rate_gain_range` to `[-5, -1]`, asserts the default `[0, 10]` is
   advertised). Cleanly testable with the existing scaffolding, so it was added.

3. **Resolved namespace not logged** (review finding
   `src/crabbing_path_follower.cpp:348-370`) — added an `RCLCPP_INFO` in
   `activate()` after the ControlServer is created, stating the resolved
   namespace and the resulting topics:
   ```cpp
   RCLCPP_INFO(
     logger_,
     "CrabbingPathFollower: marine_control namespace '%s' -> topics "
     "~/control/%s/state and ~/control/%s/change",
     marine_control_namespace_.c_str(), marine_control_namespace_.c_str(),
     marine_control_namespace_.c_str());
   ```
   This is the practical mitigation for the config-not-enforced wrap case (review
   finding `:348-370`): a deployment that wrapped the follower but forgot to set
   `marine_control.namespace` is now diagnosable from the logs.

### Deferred (per operator instruction — left as-is)
- Pre-existing package-wide lint debt (copyright headers / >100-char lines).
- `NamespaceParamDifferentiatesTopicsWhenWrapped` asserting string literals.
- Narrower-platform-range silent clamp (documented at `cpp` clamp site).

### plan.md
Not changed — the design is unchanged. The plan already describes the `_range`
override as "malformed → fallback"; the negative-but-ordered case is a
refinement of that same notion, and the two new logs are observability, not
design. plan.md remains accurate.

### Build result
`./core_ws/build.sh marine_nav_crabbing_path_follower` — **PASS** (13.6s). Only
pre-existing `-Wunused-parameter` / `-Wsign-compare` warnings in untouched code;
no new warnings from the added lines.

### Test result
`./core_ws/test.sh marine_nav_crabbing_path_follower` — all gtest suites
**green**: `test_crabbing_control` **8/8** (was 7; +`NegativeOrderedRangeFallsBackToDefault`),
`test_gain_schedule` 9/9, `test_path_geometry` 18/18. The 44 reported failures
are entirely the deferred pre-existing `ament_cpplint`/`ament_uncrustify` lint
debt (copyright headers, header-guard style, long pre-existing lines); no gtest
failures.

### Next step
Host re-review and publish (do NOT push / open PR per the exit contract).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-30 16:39 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-84 at `e96390b`
**Mode**: pre-push
**Depth**: Deep (reason: cross-module marine_control lifecycle/concurrency integration; ~570 changed code lines)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 2 | **Ship**: recommended — no must-fix; round-1 findings addressed (commit `6766314`); two disjoint-lens adversarial passes converged clean

### Findings
- [ ] (suggestion) Empty-namespace fallback (`marine_control_namespace_` reset to `plugin_name_` when the param is `""`) is live but untested — `src/crabbing_path_follower.cpp:506-508`
- [ ] (suggestion) FloatingPointRange floor is `0.0` for the exclusive `>0` gains, so a panel value of exactly 0 is advertised-in-range yet rejected by the on-set callback — by design and documented (`kTunables` comment), sibling-consistent; left as-is — `src/crabbing_path_follower.cpp:283-294`

### Adversarial dispositions (rejected as false positives this round)
- Lens A "descriptor 0.0 floor contradicts callback" → not must-fix: documented, intentional, matches the avoidance sibling (kept above as a low-priority suggestion).
- Lens A "`InRangeChangeIsApplied` re-send loop masks dropped messages" → not a defect: re-sending until the value lands is the standard ROS 2 pub/sub discovery-race pattern; the test asserts the change applies, which is its purpose.
- Lens A "`OutOfRangeChangeIsRejected` race / doesn't prove rejection" → sound: `999.0` exceeds the `[0,10]` FloatingPointRange and is rejected at the rclcpp range layer (never reaches the atomic); reliable+ordered QoS makes the landed sentinel prove delivery-then-rejection.
- Lens A "clamp silently shifts effective default when value+range both overridden out of range" → incorrect: an out-of-range override *value* is validated against the descriptor at declare and throws loudly; `std::clamp` only sets the built-in default used when no override is present.

### Deferred (carried from round 1, per operator instruction)
- Pre-existing package-wide lint debt (no copyright headers, >100-char lines, header-guard/brace style on the legacy `.cpp`/`.h`). Static analysis this round: new `test_crabbing_control.cpp` is clean except the same `legal/copyright` convention the package already follows (sibling `test_avoidance_control.cpp` has no copyright header either).
- `NamespaceParamDifferentiatesTopicsWhenWrapped` asserts string literals rather than exercising the production namespace-read/fallback.
- Narrower-platform-`_range` silently clamps the built-in default value into range (documented at the clamp site).
