# Plan: CA safety node — speed-scaled yaw-preserving slowdown + reverse-assisted stop

## Issue

https://github.com/rolker/unh_marine_navigation/issues/64

## Context

On EchoBoat 240 (vectored thrust) a hard emergency stop zeroes yaw authority, so the
boat gets stuck pointed at an obstacle. This node **replaces** the nav2 Collision Monitor
(confirmed) with a graduated **safety brake** (not a maneuvering/escape node): it slows
earlier and proportionally to speed while *preserving the avoider's yaw authority*, then
brakes to a stop with reverse thrust over a short distance. It also **takes over the
Collision Monitor's zone visualization** so CAMP's existing yellow (slowdown) / red (stop)
boxes keep rendering — with the yellow box now sized live by the speed-scaled zone. CAMP draws each box as an
**outline when idle and a translucent fill when active**, where "active" comes from a
separate `nav2_msgs/CollisionMonitorState` message — so the node publishes that too, not
just the polygons.
cmd_vel chain wiring lives in config, not this repo — so this PR delivers the **node +
tests only**; per-240 wiring is config (rolker/seafloor_echoboat_project11#43).

## Approach

1. **New package `marine_nav_ca_safety_node`**, mirroring the `marine_nav_utilities`
   standalone-node pattern (ament_cmake, `package.xml` format 2, BSD, C++17).
2. **ROS-free core header `ca_safety.h`** — pure, gtest-able functions: slowdown length
   `L = clamp(v·T + L_min, L_min, L_max)` (≡ time-to-collision); nearest forward-sector
   obstacle range from points already in base frame; twist modulation
   {slowdown scale that keeps `linear.x` at/above a positive floor; pass `angular.z`
   through; stop-zone → reverse-brake setpoint}; zone-polygon geometry (for viz);
   validation predicates (finite + domain). Mirrors the `costmap_window.h` predicate style.
3. **Node class `ca_safety_node.h`** (`rclcpp::Node`): subs `cmd_vel_in` (TwistStamped),
   pointcloud (PointCloud2, tf2 → `base_frame`), odom (Odometry, forward-speed feedback);
   pub `cmd_vel_out` (TwistStamped). On each cmd_vel: derive zone from latest cloud range
   + measured speed, modulate, publish. Reverse braking is **closed-loop on odom speed to
   ~0** (no sustained reverse), capped by `reverse_distance`/`reverse_duration`. The
   `cancel_yaw_during_reverse` **dynamic** param toggles whether `angular.z` is zeroed
   (straight brake — sidesteps yaw-sign-in-reverse) or passed through.
4. **Zone visualization (two parts — CAMP needs both):**
   (a) `geometry_msgs/PolygonStamped` on `collision_monitor/slowdown_polygon` (sized live
   by the speed-scaled `L`) and `collision_monitor/stop_polygon` (fixed), in `base_frame`
   — CAMP keys box **color** off the topic name (amber=slowdown, red=stop) and the shape
   off these.
   (b) `nav2_msgs/CollisionMonitorState` on `collision_monitor_state` with `action_type`
   = highest-priority current action (`DO_NOTHING`/`SLOWDOWN`/`STOP`) — CAMP keys the
   **frame-vs-fill** off `action_type` (outline when idle, translucent fill when a zone of
   that kind is active), with a 2 s staleness watchdog → **publish every cycle**.
   Both gated by `publish_visualization` (default true).
5. **Source-loss handling** — use the last cloud until `source_timeout`; after that,
   `source_loss_behavior` (`hold` | `passthrough` | `stop`, default `passthrough` + warn)
   decides. (Configurable per discussion — `stop` reintroduces the deadlock, so not default.)
6. **Dynamic parameter validation** per the #62 `CostmapWindowNode` precedent: pre-set
   `onSetParameters` validates (finite/domain, int→double coercion), post-set applies to
   `std::atomic` members.
7. **`main()`** wrapper in `src/ca_safety_node.cpp`.
8. **Tests**: `test_ca_safety.cpp` (pure: zone sizing at speed extremes; slowdown floor
   keeps `linear.x` > 0; reverse-brake terminates at ~0; cancel-yaw toggle both ways;
   polygon geometry tracks `L`; `action_type` maps clear→`DO_NOTHING`,
   slowdown→`SLOWDOWN`, stop→`STOP`; param validation incl. NaN/inf/out-of-domain).
   `test_ca_safety_node.cpp` (param defaults, dynamic update accept/reject, message flow,
   viz polygons + `CollisionMonitorState` published). GTest, in-process.
9. **Package README** documenting params, topics, chain placement ("replaces Collision
   Monitor"), **and the safety-behavior rationale** (reverse-near-obstacles, yaw-preserving
   slowdown) — Q4: documented with the node rather than a project `docs/decisions/`.

## Files to Change

| File (new, under `marine_nav_ca_safety_node/`) | Change |
|---|---|
| `package.xml` | format 2, BSD; deps: rclcpp, geometry_msgs, sensor_msgs, nav_msgs, nav2_msgs, tf2, tf2_ros, tf2_geometry_msgs, rcl_interfaces |
| `CMakeLists.txt` | library + `ca_safety_node` executable → `lib/${PROJECT_NAME}`; gtest registration |
| `include/marine_nav_ca_safety_node/ca_safety.h` | pure logic: zone sizing, range, twist modulation, polygon geometry, validation |
| `include/marine_nav_ca_safety_node/ca_safety_node.h` | node: subs/pub (incl. viz polygon + `CollisionMonitorState` pubs), TF, param declare+validate, modulation loop |
| `src/ca_safety_node.cpp` | `main()` |
| `test/test_ca_safety.cpp` | pure-logic unit tests |
| `test/test_ca_safety_node.cpp` | node param/flow/viz tests |
| `README.md` | params, topics, chain placement, safety-behavior rationale |

## Parameters (all validated; geometry/reverse/viz toggles are dynamic)

| Param | Type / default | Validation |
|---|---|---|
| `cmd_vel_in_topic` / `cmd_vel_out_topic` | string / `cmd_vel_smoothed`, `piloting_mode/autonomous/cmd_vel` | non-empty (TwistStamped, confirmed used everywhere) |
| `pointcloud_topic` / `odom_topic` / `base_frame` | string / `collision_monitor/pointcloud`, `odom`, `base_link` | non-empty |
| `ttc_time_constant` `T` | double / 4.0 s | finite, > 0 |
| `slowdown_min_length` / `slowdown_max_length` | double / 5.0, 25.0 m | finite, > 0, min ≤ max |
| `slowdown_speed_floor` | double / small +ε (m/s) | finite, > 0 (keeps yaw authority) |
| `slowdown_width` / `stop_length` / `stop_width` | double / 6.0, 5.0, 4.0 m | finite, > 0 |
| `reverse_speed` | double / 0.5 m/s | finite, > 0 |
| `reverse_distance` / `reverse_duration` | double / 3.0 m, 4.0 s | finite, > 0 |
| `cancel_yaw_during_reverse` | bool / true | — (live toggle for on-water A/B) |
| `source_timeout` | double / 1.0 s | finite, > 0 |
| `source_loss_behavior` | string / `passthrough` | one of `hold`,`passthrough`,`stop` |
| `publish_visualization` | bool / true | — |
| `slowdown_polygon_topic` / `stop_polygon_topic` | string / `collision_monitor/slowdown_polygon`, `collision_monitor/stop_polygon` | non-empty (name carries color) |
| `state_topic` | string / `collision_monitor_state` | non-empty (`nav2_msgs/CollisionMonitorState`; CAMP discovers by type) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control & transparency | All behavior configurable; reverse-yaw toggle is a live param; **zone viz preserved** (operator keeps the yellow/red boxes, yellow now reflects speed); output cmd_vel observable |
| A change includes its consequences | Node ships with unit tests + README rationale; config wiring at seafloor#43, recovery at #67; CM-replacement (incl. viz) reconciled |
| Only what's needed | One small node package; reuses the existing param-validation pattern; viz reuses existing CAMP display (no CAMP change); pointcloud injected (no all-around perception) |
| Improve incrementally | Node here; per-240 wiring separate (#43); recovery separate (#67) |
| Test what breaks | Unit tests target the failure modes that matter (slowdown floor, reverse termination, yaw toggle, viz tracking, bad params); sim validation required before water |
| Workspace vs project separation | Project nav code in the project repo; no workspace coupling |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | New package mirrors existing `marine_nav_utilities` (format 2, BSD, ament_cmake, C++17); launch/wiring stays in config repo |
| 0002 — Worktree isolation | Yes | Work on `feature/issue-64` worktree |
| 0001 — Capture decisions | Yes | Safety-behavior rationale documented in the package README (Q4) |
| 0013 — progress.md vocabulary | Yes | `## Plan Authored` entry appended |

## Consequences

| If we change... | Also update... | In plan? |
|---|---|---|
| Add a new node (params/topics) | Package README; per-240 config | README yes; config → seafloor#43 |
| Replace the Collision Monitor (incl. viz) | nav2 chain wiring; **CAMP viz**: polygon topics (color by name) + `CollisionMonitorState` (fill) | Config → seafloor#43; reconcile #25/#27/#36 |
| New `/odom` + pointcloud deps | Topic supply (coverage decision) | External → seafloor#43 |

## Open Questions

- [ ] None open — CAMP viz mechanism confirmed against `camp` source: box **color** keyed
      off topic name (amber=slowdown, red=stop); **frame-vs-fill** driven by
      `nav2_msgs/CollisionMonitorState.action_type` (2 s staleness watchdog). Plan is
      review-plan-ready.

## Estimated Scope

**Single PR** (node + unit tests + viz) in `unh_marine_navigation`. Config wiring is a
separate PR in `seafloor_echoboat_project11` (#43); recovery BT back-off is #67. Sim
validation before any on-water trust.
