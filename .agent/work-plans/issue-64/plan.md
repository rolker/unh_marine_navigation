# Plan: CA safety node — speed-scaled yaw-preserving slowdown + reverse-assisted stop

## Issue

https://github.com/rolker/unh_marine_navigation/issues/64

## Context

On EchoBoat 240 (vectored thrust) a hard emergency stop zeroes yaw authority, so the
boat gets stuck pointed at an obstacle. This node **replaces** the nav2 Collision Monitor
with a graduated **safety brake** (not a maneuvering/escape node): it slows earlier and
proportionally to speed while *preserving the avoider's yaw authority*, then brakes to a
stop with reverse thrust over a short distance. It also takes over the CM's zone
visualization (CAMP's yellow/red boxes; the fill is driven by a separate
`nav2_msgs/CollisionMonitorState`, so the node publishes both).

**The node is platform-agnostic.** It lives in the generic `unh_marine_navigation` core,
so it uses **generic relative topic names and unprefixed frame-param defaults**; all
namespacing and frame-prefixing (the deployment's namespace + its roll/pitch-stabilized
`…/base_link_level` frame) is applied externally by the launch/config (seafloor#43). This PR delivers the
**node + tests only**.

## Approach

1. **New package `marine_nav_ca_safety`** (executable `ca_safety_node`), mirroring the
   `marine_nav_utilities` standalone-node pattern (ament_cmake, `package.xml` format 2,
   BSD, C++17). REP-144 naming: package by domain, executable by role.
2. **ROS-free core header `ca_safety.h`** — pure, gtest-able functions: slowdown length
   `L = clamp(v·T + L_min, L_min, L_max)` (≡ time-to-collision); nearest forward-sector
   obstacle range from points already in the computation frame; twist modulation
   {slowdown scale keeping `linear.x` ≥ a positive floor; pass `angular.z`; stop-zone →
   reverse-brake setpoint}; zone-polygon geometry; validation predicates (finite + domain).
   Mirrors `costmap_window.h`.
3. **Node `ca_safety_node.h`** (`rclcpp::Node`): subs `cmd_vel_in` (TwistStamped),
   pointcloud (PointCloud2), odom (Odometry); pub `cmd_vel_out` (TwistStamped).
   **Transform:** `tf2_ros::Buffer`/`Listener` + `tf2_sensor_msgs::doTransform` to bring the
   PointCloud2 into `base_frame` before range computation — the live cloud is in a
   roll/pitch-stabilized frame, and `base_frame` is a **generic param** (default
   `base_link`; deployment supplies e.g. `…/base_link_level`). On each cmd_vel: derive zone
   from latest cloud range + measured speed, modulate, publish `cmd_vel_out`. If input
   cmd_vel pauses, the node stops emitting `cmd_vel_out` (the FCU's own GUIDED watchdog
   handles a stale command, cf. #28).
4. **Reverse-assisted stop** — closed-loop on odom forward speed to ~0, with a **hard
   backstop independent of odom**: reverse ends at `reverse_distance` (integrated from
   reverse-start) or `reverse_duration`, whichever first — a mid-reverse odom dropout
   cannot cause aft runaway. `cancel_yaw_during_reverse` dynamic param, **default `true`**
   (straight brake — the safe path). **The yaw-passthrough path (`false`) ships disabled
   and is enabled only after the sim yaw-sign-in-reverse test passes** (the issue's blocker;
   wrong sign would swing the bow *into* the obstacle).
5. **Visualization — timer-driven, both parts.** An independent timer (`viz_rate`, default
   2 Hz, well inside CAMP's 2 s watchdog) publishes: (a) `geometry_msgs/PolygonStamped` on
   the slowdown/stop polygon topics (slowdown sized live by `L`), in `base_frame` — CAMP
   colors by topic name; (b) `nav2_msgs/CollisionMonitorState.action_type`
   (`DO_NOTHING`/`SLOWDOWN`/`STOP`) — CAMP's frame-vs-fill. Driving these from a timer (not
   cmd_vel) keeps fills from blanking when cmd_vel pauses. Gated by `publish_visualization`.
6. **Source-loss handling** — cloud: use the last cloud until `source_timeout`, then
   `source_loss_behavior` (`hold`|`passthrough`|`stop`, default `passthrough`+warn; `stop`
   reintroduces the deadlock). odom: covered by the reverse hard backstop (4).
7. **Explicit QoS per endpoint** (mirroring `costmap_window_node`'s QoS rationale, which
   exists precisely because implicit QoS caused a silent no-data bug, #56): pointcloud =
   **sensor-data/best-effort** (nav2 sensor sources are best-effort; a reliable sub gets
   nothing); cmd_vel in/out = reliable, depth 1, volatile (match `velocity_smoother` out /
   `echo_helm` in — a mismatch means **no helm commands**); odom = reliable, depth 5;
   polygons + state = reliable, depth 1, volatile (match CAMP's `QoS(1)`).
8. **Sole-helm-publisher invariant** — exactly one publisher on `cmd_vel_out` at any time.
   The CM must be removed from the chain/lifecycle at cutover (wiring = seafloor#43); the
   node logs a warning if it detects another publisher on `cmd_vel_out`. (CM currently runs
   as a pass-through on non-reflex boats — cutover handled per-boat in #43.)
9. **Dynamic parameter validation** per the #62 `CostmapWindowNode` precedent (pre-set
   `onSetParameters` validates finite/domain + int→double; post-set applies to `std::atomic`).
10. **`main()`** in `src/ca_safety_node.cpp`.
11. **Tests.** `test_ca_safety.cpp` (pure: zone sizing at speed extremes; slowdown floor
    keeps `linear.x` > 0; reverse-brake terminates at ~0; reverse hard backstop on odom
    loss; cancel-yaw both ways; polygon tracks `L`; `action_type` mapping; param validation
    incl. NaN/inf/domain). `test_ca_safety_node.cpp` (param defaults, dynamic accept/reject,
    viz+state published on timer). **`test_ca_safety_launch.py` (launch_testing):** QoS
    interop (best-effort cloud + cmd_vel actually flow), TF transform of a real cloud into
    `base_frame`, single-publisher on `cmd_vel_out`. **Sim gates (before water):**
    yaw-preserving slowdown at speed extremes; reverse termination / no aft runaway; escape
    when the avoider has given up; source-loss passthrough; **yaw-sign-in-reverse sign test
    (gates enabling the passthrough path)** — recorded via sim bag (ensure CA/reverse state
    is in the recording, cf. marine_simulation #62/#63).
12. **Package README**: params, topics, QoS, chain placement, the platform-agnostic naming
    note, and the safety-behavior rationale (Q4 — documented with the node).

## Files to Change

| File (new, under `marine_nav_ca_safety/`) | Change |
|---|---|
| `package.xml` | format 2, BSD; deps: rclcpp, geometry_msgs, sensor_msgs, nav_msgs, nav2_msgs (for `CollisionMonitorState` — CAMP keys off this exact type), tf2, tf2_ros, tf2_geometry_msgs, **tf2_sensor_msgs** (PointCloud2 `doTransform`), rcl_interfaces |
| `CMakeLists.txt` | library + `ca_safety_node` exec → `lib/${PROJECT_NAME}`; gtest + launch_testing registration |
| `include/marine_nav_ca_safety/ca_safety.h` | pure logic: zone sizing, range, twist modulation, polygon geometry, validation |
| `include/marine_nav_ca_safety/ca_safety_node.h` | node: subs/pub, tf transform, timer-driven viz+state, param declare+validate, modulation loop |
| `src/ca_safety_node.cpp` | `main()` |
| `test/test_ca_safety.cpp` | pure-logic unit tests |
| `test/test_ca_safety_node.cpp` | node param/flow/viz tests |
| `test/test_ca_safety_launch.py` | launch_testing: QoS / TF / single-publisher interop |
| `README.md` | params, topics, QoS, chain, platform-agnostic note, safety rationale |

## Parameters (generic/unprefixed defaults; geometry/reverse/viz toggles dynamic)

| Param | Type / default | QoS / validation |
|---|---|---|
| `cmd_vel_in_topic` / `cmd_vel_out_topic` | string / `cmd_vel_smoothed`, `piloting_mode/autonomous/cmd_vel` | reliable d1 volatile; non-empty (TwistStamped) |
| `pointcloud_topic` | string / `collision_monitor/pointcloud` | **sensor-data/best-effort** |
| `odom_topic` | string / `odom` | reliable d5 |
| `base_frame` | string / `base_link` | non-empty (generic; deployment supplies prefixed/level frame) |
| `ttc_time_constant` `T` | double / 4.0 s | finite, > 0 |
| `slowdown_min_length` / `slowdown_max_length` | double / 5.0, 25.0 m | finite, > 0, min ≤ max |
| `slowdown_speed_floor` | double / small +ε (m/s) | finite, > 0 (keeps yaw authority) |
| `slowdown_width` / `stop_length` / `stop_width` | double / 6.0, 5.0, 4.0 m | finite, > 0 |
| `reverse_speed` | double / 0.5 m/s | finite, > 0 |
| `reverse_distance` / `reverse_duration` | double / 3.0 m, 4.0 s | finite, > 0 (hard backstop, odom-independent) |
| `cancel_yaw_during_reverse` | bool / true | passthrough path (`false`) disabled until sim sign test |
| `source_timeout` | double / 1.0 s | finite, > 0 |
| `source_loss_behavior` | string / `passthrough` | one of `hold`,`passthrough`,`stop` |
| `publish_visualization` | bool / true | — |
| `viz_rate` | double / 2.0 Hz | finite, > 0 (inside CAMP's 2 s watchdog) |
| `slowdown_polygon_topic` / `stop_polygon_topic` | string / `collision_monitor/slowdown_polygon`, `…/stop_polygon` | reliable d1; name carries color |
| `state_topic` | string / `collision_monitor_state` | reliable d1 (`nav2_msgs/CollisionMonitorState`; CAMP discovers by type) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control & transparency | All behavior configurable; reverse-yaw toggle live (passthrough gated until tested); zone viz preserved (yellow reflects speed); cmd_vel observable |
| A change includes its consequences | Unit + launch_testing + README; config at seafloor#43; recovery at #67; CM-replacement + sole-publisher cutover reconciled |
| Only what's needed | One small node package; reuses param-validation pattern; reuses CAMP display; pointcloud injected |
| Improve incrementally | Node here; per-240 wiring separate (#43); recovery separate (#67) |
| Test what breaks | Pure tests + launch_testing for the risky interop (QoS, TF, single-publisher) + enumerated sim gates; reverse-yaw passthrough gated on the sign test |
| Workspace vs project separation | Project nav code; **platform-agnostic** — generic relative topics + unprefixed frame defaults, prefix/namespace applied externally |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | Mirrors `marine_nav_utilities`; **REP-144 naming** (`marine_nav_ca_safety` pkg / `ca_safety_node` exec); explicit QoS; generic frame/topic names (platform-agnostic) |
| 0002 — Worktree isolation | Yes | `feature/issue-64` worktree |
| 0001 — Capture decisions | Yes | Safety-behavior rationale in the package README |
| 0013 — progress.md vocabulary | Yes | `## Plan Authored` / `## Plan Review` entries |

## Consequences

| If we change... | Also update... | In plan? |
|---|---|---|
| Add a new node (params/topics) | Package README; per-240 config | README yes; config → seafloor#43 |
| Replace the Collision Monitor (incl. viz) | nav2 chain wiring + **sole-helm-publisher cutover** (one publisher on `cmd_vel_out`); CAMP viz (polygon topics + `CollisionMonitorState`) | Config → seafloor#43; reconcile #25/#27/#36; CM pass-through on non-reflex boats |
| New `odom` + pointcloud deps + tf transform | Topic supply + `base_frame` value (prefixed/level) | External → seafloor#43 |

## Open Questions

- [ ] (integration, seafloor#43) Confirm the cutover removes the CM cleanly (single helm
      publisher), and that CAMP's polygon coloring keys off the topic names — verify on the
      bench during wiring.

## Estimated Scope

**Single PR** (node + unit + launch_testing) in `unh_marine_navigation`. Config wiring is a
separate PR in `seafloor_echoboat_project11` (#43); recovery BT back-off is #67. Sim gates
must pass before any on-water trust (and gate the reverse-yaw-passthrough path).
