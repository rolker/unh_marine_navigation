# Agent Guide: unh_marine_navigation

> A set of ROS 2 packages that work with [Nav2](https://github.com/ros-navigation/navigation2)
> for marine robots — a BT.CPP task navigator, marine-tuned controllers,
> behaviors, and a collision-avoidance safety brake.

## Workflow

**When this repo is checked out as part of a
[ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace)**,
workflow rules (worktree vs. field mode, branch naming, etc.) are defined in
the workspace `AGENTS.md`. To determine the active mode before editing, run
the detection script **from the workspace root** (the script is not on PATH,
and `.agent/` lives at the workspace root, not inside this repo):

```bash
# From the workspace root, pass the path to this repo
.agent/scripts/field_mode.sh --describe layers/main/core_ws/src/unh_marine_navigation
```

**Standalone use** (this repo cloned alone, outside the workspace): only this
repo's own conventions apply — the workspace-level workflow rules don't bind here.

See the repo-root `AGENTS.md` for the safety-critical review context (this
stack steers real uncrewed boats).

## Package Inventory

| Package | Language | Description |
|---------|----------|-------------|
| `marine_nav_interfaces` | IDL (ament_cmake) | 3 actions (`ComputeSonarCoveragePath`, `Hover`, `RunTasks`) + 3 msgs (`TaskInformation`, `TaskInformationList`, `TaskFeedback`) |
| `marine_nav_utilities` | C++ | Shared lib (`corridor_solver`, `costmap_window`, `gz4d` geo/matrix headers) + `costmap_window_node` executable |
| `marine_nav_tasks` | C++ + Python | Task / TaskList model over `TaskInformation`; C++ lib + a `marine_nav_tasks` Python module |
| `marine_nav_behaviors` | C++ | Nav2 behavior plugin: `marine_nav_behaviors::Hover` (`nav2_core::Behavior`), a virtual-anchor station-keep |
| `marine_nav_behavior_tree` | C++ | BT.CPP plugin lib (`libmarine_nav_behavior_tree_plugins`) of task/path BT nodes + `generate_..._nodes_xml` executable |
| `marine_nav_bt_task_navigator` | C++ | `marine_nav_bt_task_navigator::TaskNavigator` (`nav2_core::NavigatorBase`) — runs a list of nav tasks |
| `marine_nav_ca_safety` | C++ | Collision-avoidance safety brake (`ca_safety_node`), a nav2 Collision Monitor replacement |
| `marine_nav_crabbing_path_follower` | C++ | `CrabbingPathFollower` (`nav2_core::Controller`) — path tracking with wind/current crab compensation |
| `marine_nav_avoidance_controller` | C++ | `AvoidanceController` (`nav2_core::Controller`) — decorator that reshapes the path around obstacles, delegates to an inner controller |

## Repository Layout

```
unh_marine_navigation/
├── marine_nav_interfaces/          # action/ and msg/ definitions, ament_cmake
├── marine_nav_utilities/           # src/ include/marine_nav_utilities/{,gz4d/}
├── marine_nav_tasks/               # src/ (C++) + marine_nav_tasks/ (Python module)
├── marine_nav_behaviors/           # src/hover.cpp; behavior_plugin.xml
├── marine_nav_behavior_tree/       # src/plugins/{action,condition,decorator}/; bt_register_nodes.cpp
├── marine_nav_bt_task_navigator/   # src/task_navigator.cpp; navigator_plugin.xml; behavior_trees/run_tasks.xml
├── marine_nav_ca_safety/           # src/ca_safety_node.cpp; header-only ca_safety lib
├── marine_nav_crabbing_path_follower/  # src/crabbing_path_follower.cpp; plugin.xml
└── marine_nav_avoidance_controller/    # src/avoidance_controller.cpp; plugin.xml
```

## Architecture Overview

The stack layers on Nav2. `marine_nav_interfaces` defines the shared vocabulary:
a `TaskInformation` (id/type/priority/poses/`data` YAML/status) and the
`RunTasks` action carrying a `TaskInformation[]`. `marine_nav_tasks` wraps those
messages into a nested Task/TaskList tree (slash-delimited ids give parent/child
tasks) in both C++ and Python.

`marine_nav_bt_task_navigator` is a `nav2_core::NavigatorBase` plugin that
executes the `RunTasks` action by ticking a behavior tree
(`behavior_trees/run_tasks.xml`). The tree is built from BT.CPP nodes registered
in `marine_nav_behavior_tree` (`bt_register_nodes.cpp`) — action nodes such as
`SetPathFromTask`, `GetSubTasks`, `PredictStoppingPose`, `SetControllerSpeed`,
`HoverAction`, `SonarCoverageAction`; conditions like `AllTasksDoneCondition`,
`RobotOnPath`, `PathEmptyCondition`; and the `RestartOnTaskChange` decorator.

Motion is realized by two `nav2_core::Controller` plugins:
`CrabbingPathFollower` (path tracking with crab compensation) and
`AvoidanceController` (a decorator that reshapes the path around obstacles over
the local costmap, then delegates to an inner controller). `marine_nav_behaviors`
supplies the `Hover` behavior for station-keeping. `marine_nav_ca_safety`
(`ca_safety_node`) is a standalone safety brake that scales speed / stops for
collision avoidance, replacing the nav2 Collision Monitor. `marine_nav_utilities`
provides the shared corridor solver, a costmap window, and the `gz4d` geodetic
math headers used across the controllers.

## Key Files to Read First

1. `marine_nav_interfaces/msg/TaskInformation.msg` — the core task vocabulary (id/type/priority/poses/data)
2. `marine_nav_interfaces/action/RunTasks.action` — how tasks enter the navigator
3. `marine_nav_bt_task_navigator/src/task_navigator.cpp` + `behavior_trees/run_tasks.xml` — the navigator entry point and its default tree
4. `marine_nav_behavior_tree/src/bt_register_nodes.cpp` — authoritative list of registered BT node names
5. `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp` — main controller; per-pose-timestamp speed derivation
6. `marine_nav_avoidance_controller/src/avoidance_controller.cpp` — decorator controller + corridor solve
7. `marine_nav_ca_safety/include/marine_nav_ca_safety/ca_safety_node.h` — safety-brake node (node name `ca_safety`)

## Build & Test

Layer workspace is `core_ws`.

```bash
# From the layer workspace directory: layers/main/core_ws/
colcon build --symlink-install --packages-select marine_nav_crabbing_path_follower
# Testing requires setup.bash sourced in the same shell
source ../../../.agent/scripts/setup.bash \
  && colcon test --packages-select marine_nav_crabbing_path_follower \
  && colcon test-result --verbose
```

Known build issues or special requirements:
- `marine_nav_behavior_tree` builds a BT.CPP plugin lib
  (`libmarine_nav_behavior_tree_plugins`) plus a `generate_..._nodes_xml`
  helper executable; `marine_nav_utilities` builds a `costmap_window_node`
  executable and `marine_nav_ca_safety` a `ca_safety_node` executable.
- `marine_nav_tasks` and `marine_nav_ca_safety` builds bring in
  `marine_control` / `marine_control_interfaces` — build those first (see below).

## Cross-Layer Dependencies

`marine_control` and `marine_control_interfaces` come from a separate source
repo ([github.com/rolker/marine_control](https://github.com/rolker/marine_control),
`jazzy`) that must be present in the workspace. Everything else
(`nav2_*`, `behaviortree_cpp`, `tf2_*`, `control_toolbox`, standard msgs) is a
rosdep/system dependency.

| Package | Depends On | Source | What It Uses |
|---------|-----------|--------|--------------|
| `marine_nav_ca_safety` | `marine_control` | marine_control repo | control interface; `marine_control_interfaces` (test) |
| `marine_nav_crabbing_path_follower` | `marine_control` | marine_control repo | control interface + parameter descriptors; `marine_control_interfaces` (test) |
| `marine_nav_avoidance_controller` | `marine_control` | marine_control repo | control interface; `marine_control_interfaces` (test) |

Internal chain (within this repo): `interfaces` → `tasks` → `behavior_tree` →
`bt_task_navigator`; `utilities` underpins `tasks`, `behavior_tree`, and both
controllers.

## Common Pitfalls

- **Stale BT plugin `.so` via SONAME shadowing.** Adding a *new* BT node in a
  worktree can load the stale main-tree
  `libmarine_nav_behavior_tree_plugins.so` at runtime instead of the freshly
  built one. Prepend the worktree's `build/marine_nav_behavior_tree` to
  `LD_LIBRARY_PATH` so the new library wins.
- **Per-pose path timestamps are load-bearing.** `CrabbingPathFollower` derives
  trajectory speed from per-pose stamps on the followed path
  (`crabbing_path_follower.cpp` ~line 924); changing pose timestamps changes
  boat speed. The commanded `default_speed` parameter is handled separately from
  the timestamp-derived trajectory speed.
- **`default_speed` YAML type is validated.** The controller accepts both
  `PARAMETER_DOUBLE` and `PARAMETER_INTEGER` for `default_speed` (YAML
  `default_speed: 1` parses as an int) and falls back / re-sets on an invalid
  value — don't "fix" this by narrowing the type.
- **BT node names are registration strings, not class names.** The name used in
  `run_tasks.xml` is the string passed to `registerNodeType<>` in
  `bt_register_nodes.cpp` (e.g. `AllTasksDoneCondition`, `HasSubTasksCondition`),
  which differs from the C++ class name — check that file, don't infer.
- **Marine control rates are deliberate.** ~10 Hz control/update rates and slow
  vehicle response are correct for these vessels; don't flag them against
  small-robot (20+ Hz) expectations.
