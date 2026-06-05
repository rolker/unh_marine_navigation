# marine_nav_ca_safety

A collision-avoidance **safety brake** for marine vehicles that replaces the
[nav2 Collision Monitor](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html)
in the velocity command chain. It sits between the velocity smoother and the helm:
it modulates the commanded twist to avoid obstacles, and republishes the Collision
Monitor's CAMP zone visualization.

## What it does

`ca_safety_node` subscribes to the velocity command, an obstacle point cloud, and
odometry, and republishes a modulated velocity command:

1. **Speed-scaled, yaw-preserving slowdown.** As an obstacle enters a forward
   corridor whose length grows with speed (`L = speed * ttc_time_constant +
   slowdown_min_length`, clamped), the forward speed is scaled down — but never
   below `slowdown_speed_floor`, so a vectored-thrust hull keeps the thrust it
   needs to *turn*. Yaw is passed through untouched. This is what lets the
   deliberative avoider steer clear before the boat reaches the stop box.
2. **Reverse-assisted stop.** In a close stop box the node brakes to a stop with
   reverse thrust (closed-loop on odometry speed), with a hard
   `reverse_distance` / `reverse_duration` backstop that is independent of
   odometry — so an odometry dropout mid-reverse cannot cause an aft runaway.

It is **not** a maneuvering/escape node: it brakes, it does not try to turn the
boat out of trouble. Recovering from a full stop is left to the avoider (which the
yaw-preserving slowdown keeps fed) and to future work.

## Why reverse, and why yaw stays alive (safety rationale)

On a vectored-thrust boat, yaw is produced by thrust × steering — a hard stop
zeroes thrust and therefore the ability to turn, leaving the boat stuck pointed at
the obstacle. So this node (a) never zeroes forward speed while slowing (keeps yaw
authority) and (b) uses reverse thrust to brake in a short distance. By default
`cancel_yaw_during_reverse` is `true` (a straight brake), because passing yaw
through *during reverse* requires the steering sign to flip for the reversed
direction; the pass-through path stays disabled until that sign behavior is
verified in simulation.

## Platform-agnostic

This package is hull-agnostic: all topic names are **relative** and all frame
names come from parameters with **generic, unprefixed defaults**. The deployment
applies the namespace (e.g. via `PushRosNamespace`) and the actual frame (e.g. a
roll/pitch-stabilized `…/base_link_level`) through launch/config. Nothing here is
specific to a particular boat.

## Topics

| Direction | Default topic | Type | QoS |
|---|---|---|---|
| sub | `cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | reliable, depth 1 |
| sub | `collision_monitor/pointcloud` | `sensor_msgs/PointCloud2` | **best-effort** (sensor data) |
| sub | `odom` | `nav_msgs/Odometry` | reliable, depth 5 |
| pub | `piloting_mode/autonomous/cmd_vel` | `geometry_msgs/TwistStamped` | reliable, depth 1 |
| pub | `collision_monitor/slowdown_polygon` | `geometry_msgs/PolygonStamped` | reliable, depth 1 |
| pub | `collision_monitor/stop_polygon` | `geometry_msgs/PolygonStamped` | reliable, depth 1 |
| pub | `collision_monitor_state` | `nav2_msgs/CollisionMonitorState` | reliable, depth 1 |

QoS is set **explicitly** on every endpoint: the point-cloud subscription is
best-effort to match the reflex cloud, and the cmd_vel/viz endpoints are reliable
to match the smoother/helm and CAMP. (Implicit QoS has silently lost data in this
stack before.)

### CAMP visualization

CAMP draws the slowdown (amber) and stop (red) boxes from the two
`PolygonStamped` topics (colored by topic name) and fills them when active based
on `collision_monitor_state.action_type`. The state is published on an
independent timer (`viz_rate`, default 2 Hz) so the fill does not blank under
CAMP's 2 s staleness watchdog when cmd_vel pauses. The slowdown polygon is resized
each tick to the current speed-scaled length.

## Parameters

Geometry/reverse tunables and the two toggles are **dynamically updatable**
(validated: finite and > 0); topics, frames, `source_loss_behavior`, and
`viz_rate` are read at startup.

| Parameter | Default | Notes |
|---|---|---|
| `ttc_time_constant` | 4.0 s | slowdown length = speed·T + min |
| `slowdown_min_length` / `slowdown_max_length` | 5.0 / 25.0 m | |
| `slowdown_speed_floor` | 0.1 m/s | forward speed kept during slowdown (yaw authority) |
| `slowdown_width` | 6.0 m | |
| `stop_length` / `stop_width` | 5.0 / 4.0 m | |
| `reverse_speed` | 0.5 m/s | reverse-brake setpoint magnitude |
| `reverse_distance` / `reverse_duration` | 3.0 m / 4.0 s | odom-independent hard backstop |
| `reverse_clear_debounce` | 1.0 s | sustained-clear time before a reverse episode resets its backstop (flicker guard) |
| `stop_speed_eps` | 0.05 m/s | speed below which the boat counts as stopped (ends reverse brake) |
| `cancel_yaw_during_reverse` | true | passthrough (false) gated on the sim sign test |
| `source_timeout` | 1.0 s | max cloud/odom age before considered lost |
| `source_loss_behavior` | `passthrough` | `hold` \| `passthrough` \| `stop` |
| `publish_visualization` | true | |
| `viz_rate` | 2.0 Hz | inside CAMP's 2 s watchdog |
| `base_frame` | `base_link` | generic; deployment supplies the prefixed/level frame |
| `*_topic` | see table above | all relative |

## Chain placement

This node **replaces** the Collision Monitor, so exactly one of the two may
publish the helm topic at a time. The per-boat wiring (removing the Collision
Monitor, pointing the smoother output and reflex cloud at this node, supplying the
namespaced/level frame) lives in the deployment config repo, not here.
