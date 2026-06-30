# Plan: Expose crabbing-path-follower parameters through marine_control

## Issue

https://github.com/rolker/unh_marine_navigation/issues/84

## Context

`CrabbingPathFollower` has a live-tuning callback registered in `configure()` that
updates `std::atomic<double>` members for 10 parameters. Those atomics are read
inside `computeVelocityCommands()` on the controller thread. The callback fires on
`ros2 param set` / BT `SetParameters` calls; there is no `marine_control::ControlServer`,
so `rqt_marine_control` and the `udp_bridge` change channel cannot reach these params
from the operator station.

Sibling controllers `marine_nav_avoidance_controller` and `marine_nav_ca_safety` already
expose params through `ControlServer`. We mirror that pattern exactly.

## Approach

1. **Add free functions to `crabbing_path_follower.h` / `.cpp`**, following the
   `declareAvoidanceControlParams` / `bindAvoidanceControls` pattern in
   `avoidance_controller.cpp:159-223`:
   - `declareCrabbingControlParams()` declares the **9 simple tunables** via a
     `kTunables[]` table, each with a `FloatingPointRange` descriptor and an
     optional platform `<name>.<suffix>_range` startup override (same `_range`
     mechanism, integer-array-coerced). The 9: `heading_rate_gain`, `max_yaw_rate`,
     `lookahead_distance`, `lookahead_time`, `lookahead_min_distance`,
     `new_plan_goal_tolerance`, `cross_track_error_slew_rate`, `pid.gain_ref_speed`,
     `pid.gain_v_min`.
   - `declareCrabbingDefaultSpeed()` handles **`default_speed` separately**
     (correction B): it is already declared at `crabbing_path_follower.cpp:43`
     *before* the `read_validated` block, so a generic helper would no-op for it.
     It attaches a `FloatingPointRange` at that existing declaration site, with a
     deliberately **permissive** range (`[0, 20]` m/s, matching the avoidance
     sibling's `avoid_speed` bound) so the bespoke configure-time fallback
     (`cpp:52-107`) is preserved for every physically meaningful value; only a
     self-contradictory out-of-range *override* fails loudly at declare.
   - `bindCrabbingControls()` binds `default_speed` **plus** the 9 tunables (10
     controls total).

2. **Update `configure()`**: replace the bare `default_speed` declare at `:43` with
   `declareCrabbingDefaultSpeed(node, plugin_name_)`; call
   `declareCrabbingControlParams(node, plugin_name_)` before the `read_validated`
   block (descriptors attach at first declaration; `read_validated`'s
   `declare_parameter_if_not_declared` then no-ops the declaration but still reads +
   validates the value into each atomic — idempotent).

3. **Add `control_server_` member** (`std::shared_ptr<marine_control::ControlServer>`)
   and a `marine_control_namespace_` member to the header; add
   `#include "marine_control/control_server.hpp"`.

4. **Wrap-case namespace (correction A)**: declare
   `<plugin_name_>.marine_control.namespace` (string descriptor) in `configure()`,
   defaulting to `plugin_name_`. `activate()` builds the `ControlServer` topics as
   `~/control/<namespace>/state|change`. Unset → byte-identical to the historical
   standalone layout. When this follower is wrapped under the SAME plugin name (the
   avoidance controller configures + activates its inner controller as
   `plugin_name_` and itself advertises a `ControlServer` on
   `~/control/<plugin_name_>/...`), the deployment sets this param to a distinct
   value so the inner and wrapping servers don't collide. The bound parameter
   *names* stay `<plugin_name_>.*` regardless — only the panel channel is namespaced.

5. **`activate()`**: instantiate `ControlServer` (device_name
   `"Crabbing Path Follower (<plugin_name_>)"`, namespaced topics) and call
   `bindCrabbingControls()`. The single-threaded-executor bind-before-spin
   constraint from avoidance `:297-305` applies equally; documented inline.

6. **`deactivate()` / `cleanup()`**: `control_server_.reset()` — tears down
   heartbeat/sub while inactive (mirrors avoidance `:332` / `:285`).

7. **Dual-path comment** in `activate()`: a marine_control `change` message is
   applied by setting the bound parameter, which runs the SAME
   `add_on_set_parameters_callback` registered in `configure()` — so a panel change
   and a `ros2 param set` both land on the same `std::atomic` members and stay
   consistent. No second callback is needed.

8. **`package.xml`**: add `<depend>marine_control</depend>` and
   `<test_depend>marine_control_interfaces</test_depend>` (correction C).

9. **`CMakeLists.txt`**: add `find_package(marine_control REQUIRED)` and
   `marine_control` to the library's `ament_target_dependencies`. Add the
   `test_crabbing_control` gtest target with
   `target_link_libraries(test_crabbing_control ${PROJECT_NAME})` and
   `ament_target_dependencies(... marine_control marine_control_interfaces rclcpp
   rclcpp_lifecycle)` — mirrors the avoidance test CMake (correction C).

10. **New test `test/test_crabbing_control.cpp`**: exercise
    `declareCrabbingDefaultSpeed` / `declareCrabbingControlParams` /
    `bindCrabbingControls` against a bare `rclcpp_lifecycle::LifecycleNode` + real
    `ControlServer` (no nav2 bring-up), mirroring `test_avoidance_control.cpp`.
    Tests: all 10 controls advertised with ranges/groups; platform `_range`
    override honoured + integer-coerced + malformed-fallback; the namespace param
    differentiates topics; in-range change applied; out-of-range rejected.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_crabbing_path_follower/include/marine_nav_crabbing_path_follower/crabbing_path_follower.h` | Add marine_control include, `control_server_` + `marine_control_namespace_` members, three free-function declarations |
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` | Add `declareCrabbingDefaultSpeed` / `declareCrabbingControlParams` / `bindCrabbingControls`; update configure() (default_speed descriptor, 9-tunable declare, namespace param), activate(), deactivate(), cleanup() |
| `marine_nav_crabbing_path_follower/package.xml` | Add `<depend>marine_control</depend>` + `<test_depend>marine_control_interfaces</test_depend>` |
| `marine_nav_crabbing_path_follower/CMakeLists.txt` | `find_package(marine_control)`, add to lib `ament_target_dependencies`, add `test_crabbing_control` target (links `${PROJECT_NAME}`, deps marine_control/marine_control_interfaces/rclcpp/rclcpp_lifecycle) |
| `marine_nav_crabbing_path_follower/test/test_crabbing_control.cpp` | New gtest: declare/bind helpers + namespace + channel test |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Directly improves operator visibility — params reachable topside via rqt_marine_control |
| A change includes its consequences | package.xml + CMakeLists.txt + test all updated in same PR; dual-path comment captures invariant |
| Only what's needed | Mirrors existing sibling pattern; no new abstractions; no unused bindings |
| Test what breaks | New test exercises the binding path without full nav2 bring-up |
| Capture decisions | Comment in activate() records the dual-path atomics invariant |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 — ROS 2 conventions | Yes | `package.xml` follows ROS 2 dep format; `CMakeLists.txt` uses `find_package` + `ament_target_dependencies` pattern |
| External marine_control ADR-0003 (D4–D6) — *the marine_control device-control ADR (cf. `test_avoidance_control.cpp:2` "unh_marine_autonomy#140 / ADR-0003"), NOT this workspace's ADR-0003 (`workspace-infrastructure-is-project-agnostic`)* | Yes | ControlServer constructed in activate(), reset in deactivate()/cleanup(); namespaced topics (default = plugin name); mirrors the avoidance sibling exactly |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `configure()` param declarations (add descriptors) | `read_validated` block — no-op for already-declared; no change needed | Yes — verified idempotent |
| `default_speed` gets a `FloatingPointRange` | Preserve the existing graceful fallback (`cpp:52-107`): keep the range permissive (`[0, 20]` m/s) so only a self-contradictory out-of-range *override* throws at declare; in-range, integer, and bool/wrong-type values still route through the fallback | Yes — `declareCrabbingDefaultSpeed` (correction B) |
| Add `ControlServer` activate/deactivate | Ensure `cleanup()` also resets it (nav2 can call cleanup without prior deactivate) | Yes — step 6 |
| A crabbing follower is wrapped under the SAME plugin name (avoidance controller) | Differentiate the inner's marine_control channel via `<name>.marine_control.namespace` so the inner and wrapping `ControlServer`s don't collide on identical `~/control/<ns>/state\|change` topics; default = plugin name keeps standalone byte-identical | Yes — `marine_control_namespace_` (correction A). Deployment config sets the inner's namespace when wrapping. |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
