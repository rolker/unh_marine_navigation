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

1. **Add free functions `declareCrabbingControlParams()` and `bindCrabbingControls()`**
   to `crabbing_path_follower.h` / `crabbing_path_follower.cpp`, following the
   `declareAvoidanceControlParams` / `bindAvoidanceControls` pattern in
   `avoidance_controller.cpp:186-223`. Each operator-relevant parameter gets a
   `FloatingPointRange` descriptor with sensible defaults and optional platform
   `<name>.<suffix>_range` startup override (same `_range` mechanism).
   Params to expose (9): `default_speed`, `heading_rate_gain`, `max_yaw_rate`,
   `lookahead_distance`, `lookahead_time`, `lookahead_min_distance`,
   `new_plan_goal_tolerance`, `cross_track_error_slew_rate`, `pid.gain_ref_speed`,
   `pid.gain_v_min`.

2. **Update `configure()`**: call `declareCrabbingControlParams(node, plugin_name_)`
   before the existing `read_validated` block. The free function declares params with
   descriptors; `read_validated` and the `default_speed` guard become no-ops for
   already-declared params (idempotent).

3. **Add `control_server_` member** (`std::shared_ptr<marine_control::ControlServer>`)
   to the header; add `#include "marine_control/control_server.hpp"`.

4. **`activate()`**: instantiate `ControlServer` with per-plugin topics
   (`~/control/<plugin_name_>/state` / `change`) and call `bindCrabbingControls()`.
   Mirrors avoidance controller `activate()` at `:306-313`. The single-threaded
   executor constraint from that block's comment applies equally here.

5. **`deactivate()`**: `control_server_.reset()` — tears down heartbeat/sub while
   inactive; mirrors avoidance controller `:332`.

6. **`cleanup()`**: `control_server_.reset()` — safety net matching `:285`.

7. **Add code comment** in `activate()` explaining the dual-path invariant: both a
   marine_control `change` message and `ros2 param set` reach the same atomics via the
   existing `add_on_set_parameters_callback`. No second callback is needed.

8. **`package.xml`**: add `<depend>marine_control</depend>`.

9. **`CMakeLists.txt`**: add `find_package(marine_control REQUIRED)` and `marine_control`
   to `ament_target_dependencies`. Add `test_crabbing_control` gtest target.

10. **New test `test/test_crabbing_control.cpp`**: exercise `declareCrabbingControlParams`
    / `bindCrabbingControls` against a bare `rclcpp_lifecycle::LifecycleNode` + real
    `ControlServer` (no nav2 bring-up), following `test_avoidance_control.cpp`.
    Tests: all params advertised with ranges/groups; in-range change applied; out-of-range
    rejected.

## Files to Change

| File | Change |
|------|--------|
| `marine_nav_crabbing_path_follower/include/marine_nav_crabbing_path_follower/crabbing_path_follower.h` | Add marine_control include, `control_server_` member, free function declarations |
| `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` | Add `declareCrabbingControlParams` / `bindCrabbingControls`; update configure(), activate(), deactivate(), cleanup() |
| `marine_nav_crabbing_path_follower/package.xml` | Add `<depend>marine_control</depend>` |
| `marine_nav_crabbing_path_follower/CMakeLists.txt` | `find_package(marine_control)`, add to `ament_target_dependencies`, add test target |
| `marine_nav_crabbing_path_follower/test/test_crabbing_control.cpp` | New gtest: declare/bind helpers + channel test |

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
| ADR-0003 (marine_control D4–D6) | Yes | ControlServer constructed in activate(), reset in deactivate()/cleanup(); per-plugin topics |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `configure()` param declarations (add descriptors) | `read_validated` block — no-op for already-declared; no change needed | Yes — verified idempotent |
| Add `ControlServer` activate/deactivate | Ensure `cleanup()` also resets it (nav2 can call cleanup without prior deactivate) | Yes — step 6 |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR.
