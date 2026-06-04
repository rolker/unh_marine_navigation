# Plan: CA safety node — speed-scaled yaw-preserving slowdown + reverse-assisted stop

## Issue

https://github.com/rolker/unh_marine_navigation/issues/64

## Context

On EchoBoat 240 (vectored thrust) a hard emergency stop zeroes yaw authority, so the
boat gets stuck pointed at an obstacle. This node replaces the nav2 Collision Monitor
with a graduated **safety brake** (not a maneuvering/escape node): it slows earlier and
proportionally to speed while *preserving the avoider's yaw authority*, then brakes to a
stop with reverse thrust over a short distance. cmd_vel chain wiring lives in config, not
this repo — so this PR delivers the **node + tests only**; per-240 wiring is config
(rolker/seafloor_echoboat_project11#43).

## Approach

1. **New package `marine_nav_ca_safety_node`**, mirroring the `marine_nav_utilities`
   standalone-node pattern (ament_cmake, `package.xml` format 2, BSD, C++17).
2. **ROS-free core header `ca_safety.h`** — pure, gtest-able functions: slowdown length
   `L = clamp(v·T + L_min, L_min, L_max)` (≡ time-to-collision); nearest forward-sector
   obstacle range from points already in base frame; twist modulation
   {slowdown scale that keeps `linear.x` at/above a positive floor; pass `angular.z`
   through; stop-zone → reverse-brake setpoint}; validation predicates (finite + domain).
   Mirrors the `costmap_window.h` predicate style.
3. **Node class `ca_safety_node.h`** (`rclcpp::Node`): subs `cmd_vel_in` (TwistStamped),
   pointcloud (PointCloud2, tf2 → `base_frame`), odom (Odometry, forward-speed feedback);
   pub `cmd_vel_out` (TwistStamped). On each cmd_vel: derive zone from latest cloud range
   + measured speed, modulate, publish. Reverse braking is **closed-loop on odom speed to
   ~0** (no sustained reverse), capped by `reverse_distance`/`reverse_duration`. The
   `cancel_yaw_during_reverse` **dynamic** param toggles whether `angular.z` is zeroed
   (straight brake — sidesteps yaw-sign-in-reverse) or passed through.
4. **Dynamic parameter validation** per the #62 `CostmapWindowNode` precedent: pre-set
   `onSetParameters` validates (finite/domain, int→double coercion), post-set applies to
   `std::atomic` members.
5. **`main()`** wrapper in `src/ca_safety_node.cpp`.
6. **Tests**: `test_ca_safety.cpp` (pure: zone sizing at speed extremes; slowdown floor
   keeps `linear.x` > 0; reverse-brake terminates at ~0; cancel-yaw toggle both ways;
   param validation incl. NaN/inf/out-of-domain). `test_ca_safety_node.cpp` (param
   defaults, dynamic update accept/reject, basic message flow). GTest, in-process.
7. **Package README** (params, topics, chain placement, "replaces Collision Monitor").

## Files to Change

| File (new, under `marine_nav_ca_safety_node/`) | Change |
|---|---|
| `package.xml` | format 2, BSD; deps: rclcpp, geometry_msgs, sensor_msgs, nav_msgs, tf2, tf2_ros, tf2_geometry_msgs, rcl_interfaces |
| `CMakeLists.txt` | library + `ca_safety_node` executable → `lib/${PROJECT_NAME}`; gtest registration |
| `include/marine_nav_ca_safety_node/ca_safety.h` | pure logic: zone sizing, range, twist modulation, validation predicates |
| `include/marine_nav_ca_safety_node/ca_safety_node.h` | node class: subs/pub, TF, param declare+validate, modulation loop |
| `src/ca_safety_node.cpp` | `main()` |
| `test/test_ca_safety.cpp` | pure-logic unit tests |
| `test/test_ca_safety_node.cpp` | node param/flow tests |
| `README.md` | params, topics, chain placement |

## Parameters (all validated; geometry/reverse are dynamic)

| Param | Type / default | Validation |
|---|---|---|
| `cmd_vel_in_topic` / `cmd_vel_out_topic` | string / `cmd_vel_smoothed`, `piloting_mode/autonomous/cmd_vel` | non-empty |
| `pointcloud_topic` / `odom_topic` / `base_frame` | string / `collision_monitor/pointcloud`, `odom`, `base_link` | non-empty |
| `ttc_time_constant` `T` | double / 4.0 s | finite, > 0 |
| `slowdown_min_length` / `slowdown_max_length` | double / 5.0, 25.0 m | finite, > 0, min ≤ max |
| `slowdown_speed_floor` | double / small +ε (m/s) | finite, > 0 (keeps yaw authority) |
| `slowdown_width` / `stop_length` / `stop_width` | double / 6.0, 5.0, 4.0 m | finite, > 0 |
| `reverse_speed` | double / 0.5 m/s | finite, > 0 |
| `reverse_distance` / `reverse_duration` | double / 3.0 m, 4.0 s | finite, > 0 |
| `cancel_yaw_during_reverse` | bool / true | — (live toggle for on-water A/B) |
| `source_timeout` | double / 1.0 s | finite, > 0 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control & transparency | All behavior is configurable; reverse-yaw toggle is a live param; output cmd_vel is observable (a diagnostics topic is a possible follow-up) |
| A change includes its consequences | Node ships with unit tests; config wiring tracked at seafloor#43, recovery at #67; CM-replacement reconciliation called out |
| Only what's needed | One small node package; reuses the existing param-validation pattern; no all-around perception (pointcloud injected) |
| Improve incrementally | Node here; per-240 wiring separate (#43); recovery separate (#67) |
| Test what breaks | Unit tests target the failure modes that matter (slowdown floor, reverse termination, yaw toggle, bad params); sim validation required before water |
| Workspace vs project separation | Project nav code in the project repo; no workspace coupling |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | New package mirrors existing `marine_nav_utilities` (format 2, BSD, ament_cmake, C++17); launch/wiring stays in config repo |
| 0002 — Worktree isolation | Yes | Work on `feature/issue-64` worktree |
| 0001 — Capture decisions | Watch | Safety-behavior rationale lives in #64/PR; project repo has no `docs/decisions/` — see Open Questions |
| 0013 — progress.md vocabulary | Yes | `## Plan Authored` entry appended |

## Consequences

| If we change... | Also update... | In plan? |
|---|---|---|
| Add a new node (params/topics) | Package README; per-240 config | README yes; config → seafloor#43 |
| Replace the Collision Monitor | nav2 chain wiring (CM in/out, velocity_smoother placement) | Config → seafloor#43; reconcile #25/#27/#36 |
| New `/odom` + pointcloud deps | Topic supply (coverage decision) | External → seafloor#43 |

## Open Questions

- [ ] **Replace vs augment** the Collision Monitor — plan assumes *replace*; confirm.
- [ ] **Stale/absent pointcloud** behavior — passthrough, hold-last, or fail-safe stop? (safety decision; note stop kills yaw).
- [ ] **cmd_vel message type** — assumed TwistStamped (CM ran stamped); confirm against the live chain.
- [ ] **Capture the safety decision** — record reverse-near-obstacles rationale in PR, or adopt a project `docs/decisions/`?

## Estimated Scope

**Single PR** (node + unit tests) in `unh_marine_navigation`. Config wiring is a separate
PR in `seafloor_echoboat_project11` (#43); recovery BT back-off is #67. Sim validation
before any on-water trust.
