# Plan: Build on ROS 2 Rolling, first slice: marine_nav_interfaces, marine_nav_utilities, marine_nav_tasks

## Issue

https://github.com/rolker/unh_marine_navigation/issues/113 (part of rolker/unh_marine_autonomy#415)

## Context

`rolling` and `jazzy` are both at `60e4063`, so the port branches from `rolling` and every fix backports to
`jazzy` by cherry-pick. Scope is three packages; the six Nav2 packages are left for their own issue and must
never be built without a selector.

Baseline (Rolling, GCC 15, CMake 4, Python 3.14; `./build.sh --continue-on-error --packages-up-to <three>`):

| Package | First failure |
|---|---|
| `marine_nav_interfaces` | builds (only the CMake `<3.10` deprecation warning); `./test.sh` runs 0 tests: its `CMakeLists.txt` never calls `ament_lint_auto`, so the `test_depend`s are dead and `ament_xmllint` never runs on its manifest |
| `marine_nav_utilities` | `CMake Error at CMakeLists.txt:70 (ament_target_dependencies): Unknown CMake command "ament_target_dependencies".` |
| `marine_nav_tasks` | not processed (depends on utilities) |

What a scratch copy (outside the repository) showed behind each, applying the fixes below in order:
cause 1 deleted -> `utilities.h:8:10: fatal error: tf2/utils.h: No such file or directory` -> cause 2 applied ->
all three build with no compiler warning (`-Wall -Wextra -Wpedantic`) -> `colcon test` with `-LE linter`:
`35 tests, 0 errors, 0 failures, 0 skipped`. Nothing else failed; no cause 3-8, 12-14 applies (no `setup.py`,
no `.warn(`, no Nav2/geodesy/GDAL, no `tf2_ros`).

## Approach

Commits on `feature/issue-113`, one fix per commit, each naming its #415 cause. The scratch result above is
the result of applying 1 to 5 together; commit 1 alone does not build (cause 2 is the next failure), so
review them as a sequence. B = for the hand backport to `jazzy`, in this order.

1. **Cause 1 (B)** — delete both `ament_target_dependencies` calls (`marine_nav_utilities/CMakeLists.txt:70-76`,
   `marine_nav_tasks/CMakeLists.txt:61-64`, with the blank line left behind). Confirmed redundant three ways:
   (a) Jazzy's own `ament_target_dependencies` (read at `ament/ament_cmake` branch `jazzy`) links each package's
   `${pkg}_TARGETS` when it has them, and a check inside the call on the scratch copy found every such target
   (geometry_msgs, nav_msgs, `rclcpp::rclcpp`, `tf2::tf2`, `tf2_geometry_msgs::tf2_geometry_msgs`;
   `marine_nav_interfaces` and `rclcpp` for tasks) already in the target's `LINK_LIBRARIES`;
   (b) `link.txt` and `flags.make` of both libraries, and every installed CMake export file, are
   byte-identical between that run and the deleted form; (c) the installed
   `export_marine_nav_utilitiesExport.cmake` still lists `rclcpp::rclcpp;tf2::tf2;tf2_geometry_msgs::...`.
   Honest limit: the Jazzy side is by reading, not by running; (b) compares against a stand-in that adds only
   what (a) says Jazzy adds, so it shows "adds nothing", not an independent Jazzy build. The unchanged
   `ament_export_dependencies` blocks stay as they are.
2. **Cause 2 (B)** — `"tf2/utils.h"` -> `"tf2/utils.hpp"` in `utilities.h:8`, `src/utilities.cpp:3`,
   `src/costmap_window.cpp:6`. These are the only removed tf2 headers the three packages compile.
   `gz4d/transforms.hpp:5` (`tf2/LinearMath/Quaternion.h`) is NOT touched: nothing compiles it (see gz4d).
3. **Cause 9 (B)** — `marine_nav_interfaces/package.xml`: move the two `<test_depend>` above
   `<member_of_group>`. Check: `ament_xmllint` fails before (`package.xml:25: Element 'test_depend': This
   element is not expected. Expected is one of ( member_of_group, export )`) and passes after.
4. **Cause 11, `marine_nav_interfaces` (B)** — add `<depend>builtin_interfaces</depend>` (used by
   `action/Hover.action:23`, `builtin_interfaces/Duration`) and `<depend>std_msgs</depend>`. `std_msgs` is NOT
   used by any `.msg`/`.action`, but `CMakeLists.txt` does `find_package(std_msgs REQUIRED)` and lists it in
   `DEPENDENCIES`, so it must be declared; dropping it from CMake would change the package's exported
   dependencies for consumers and is left to the follow-up issue.
5. **Cause 11, `marine_nav_tasks` (B)** — add `<depend>geometry_msgs</depend>` (named directly:
   `task.h:96`, `task.cpp:134,146`, `task_list.cpp:113`, only reached today through
   `marine_nav_interfaces`), `<exec_depend>rclpy</exec_depend>` and `<exec_depend>python3-yaml</exec_depend>`
   (`task.py:5,10`, `task_list.py:10` import `rclpy` and `yaml`). `package.xml` only; no CMake change, so no
   link-line change. `marine_nav_utilities` needs nothing (every include maps to a declared key).
   Keys, by `rosdep resolve` and by reading `ros/rosdistro` master (`jazzy/` and `rolling/distribution.yaml`
   list `builtin_interfaces`, `std_msgs`, `geometry_msgs`, `rclpy`, `yaml_cpp_vendor`, `tf2`,
   `tf2_geometry_msgs`, `nav_msgs`, `rcl_interfaces`, `rclcpp_lifecycle`, `visualization_msgs`; `rosdep/python.yaml`
   `python3-yaml: ubuntu: [python3-yaml]`): `ros-jazzy-*` on noble, `ros-rolling-*` on resolute,
   `python3-yaml` on both. `release_platforms` is `ubuntu: noble` (jazzy) and `resolute` (rolling).
6. **gz4d: no code commit.** See below. Nothing in the three packages needs a change in any gz4d header.
7. **CI, commit A (B)** — `.github/workflows/ci.yml`, see CI section.
8. **CI, commit B (B, droppable)** — strict manifest check of the three in the Jazzy job.

Order of backport: 1, 2, 3, 4, 5, 7, 8 (7 and 8 apply cleanly because `jazzy`'s `ci.yml` is unchanged).
The `.agent/work-plans/issue-113/` records are carried in the pull request to `rolling` and not backported
(settled). Nothing here conflicts on cherry-pick: the files touched do not overlap between commits except
`marine_nav_interfaces/package.xml` (commits 3, 4: apply in order; the hunks do not overlap).

## gz4d headers (cause 10, host decision: fix only what a Rolling build needs)

Established by the `.d` dependency files of the scratch build (tests included): the only gz4d header any
translation unit of the three packages includes is `gz4d/vector.hpp` (from `src/utilities.cpp:2`). GCC 15
accepts it there with zero diagnostics, so **no fix is needed or planned**, including the two #418 fixes in
`geo.hpp`. No file outside the six Nav2 packages includes any other gz4d header from this package; the one
outside user is `marine_nav_crabbing_path_follower/src/crabbing_path_follower.cpp:14` (`gz4d/angles.hpp`), which
therefore stays untouched.

Left untouched, defects as GCC 15 reports them compiling each header alone
(`g++ -std=c++17 -Wall -Wextra -Wpedantic -fsyntax-only`; the second column is what remains with all
standard headers pre-included, so it is the real defect and not a missing include):

| Header | Standalone (missing includes) | Real defects |
|---|---|---|
| `angles.hpp` | `:88-90` needs `<sstream>`, `<stdexcept>` | `:124,126,127` `!*this==o` parses as `(!*this)==o` (`-Wlogical-not-parentheses`; #420's "Angle comparison"); **used by crabbing_path_follower: belongs to the six-package issue** |
| `vector.hpp` | `:34` `std::pair` (`<utility>`); `:294-295` `std::nan`, `std::isnan` (`<cmath>`) | none; compiles in `utilities.cpp` only because `utilities.h` happens to include those first |
| `matrix.hpp` | `:21` `std::pair`; `:91` `std::numeric_limits` (`<limits>`) | none |
| `box.hpp` | `:45` `isnan`; `:76,87,195,196` `std::min/max` (`<algorithm>`, `<cmath>`) | (`Box::operator==` self-call is #420) |
| `utilities.hpp` | `:68` `std::istream` (`<istream>`) | none |
| `geo.hpp` | `:208,289` `std::ostream`; `:336` `std::shared_ptr` (`<memory>`); `:389` `std::vector` | `:291` `Bounds<T>::getMin/getMax` do not exist (#418 fix 1); `:71` `expected template-name before '<'`; `:303-304,349,354` `Matrix` not declared (no `matrix.hpp` include); `:315-332` `transform`/`inverse` not declared; `:359-399` `Point`/`Vector`/`Box2d` unqualified in namespace |
| `ellipsoid.hpp` | `:23` `std::pair`; `:30,52,148-158` `sin/pow/sqrt/atan` | `:20` template parameter `EllipsoidSpecs` redeclared by a `using` (error); `:22,34,40,56,62` `Angle` not declared (no include of `angles.hpp`); `:70-95` `LongitudeSpanRadians`/`LatitudeSpanRadians`; `:101,117,127,166,232` `Point` not declared; `:247,322,327` `azimuth_distance_type` |
| `wgs84.hpp` | none of its own | `:27-31` `ReferenceFrame`, `:39-46` `Point`/`LocalENU`/`Position`/`Bounds` do not name types (needs `geo.hpp`, which does not compile) |
| `transforms.hpp` | none | `:5` `tf2/LinearMath/Quaternion.h`: removed (`.hpp` exists on both distros: `Quaternion.hpp` is in `ros2/geometry2` jazzy `tf2/include/tf2/LinearMath/`); `Angle` used unqualified in the namespace |
| `gz4d.hpp` | none | fails only through `transforms.hpp:5` |

For the follow-up issue next to #420: these headers have not compiled since the namespace change and are
installed but never built; either repair them together or stop installing them.

## Files to Change

| File | Change |
|---|---|
| `marine_nav_utilities/CMakeLists.txt` | delete `ament_target_dependencies` (commit 1) |
| `marine_nav_tasks/CMakeLists.txt` | delete `ament_target_dependencies` (commit 1) |
| `marine_nav_utilities/include/.../utilities.h`, `src/utilities.cpp`, `src/costmap_window.cpp` | `tf2/utils.hpp` (commit 2) |
| `marine_nav_interfaces/package.xml` | element order (3); `builtin_interfaces`, `std_msgs` (4) |
| `marine_nav_tasks/package.xml` | `geometry_msgs`, `rclpy`, `python3-yaml` (5) |
| `.github/workflows/ci.yml` | triggers, Rolling job (7); strict step group in Jazzy job (8) |

## CI

One file on both branches, modelled on `mru_transform`'s `ci.yml` (and `unh_marine_autonomy`'s
`ros-base-docker.yml`): triggers `push`/`pull_request` on `[jazzy, rolling]`; job ids `jazzy`
(`build-and-test (jazzy)`) and `rolling` (`build-and-test (rolling)`). The repository's ruleset on the default
branch (`jazzy`) requires a pull request but no named check, so renaming the existing job is safe. The header
comment of the file says: the six Nav2 packages do not build on Rolling, so the Rolling job covers three.

**Job `rolling`** (commit 7): `if: (github.base_ref || github.ref_name) != 'jazzy'`; `ros:rolling-ros-core`;
apt update, `ros-dev-tools git`, checkout, `rosdep init` guarded, `rosdep update`, workspace symlink
`../ws/src/unh_marine_navigation`; no `marine_control` clone; `rosdep install --ignore-src -y --from-paths
$paths` with `paths=$(colcon list --base-paths src --paths-only --packages-up-to marine_nav_interfaces
marine_nav_utilities marine_nav_tasks)` (guarded for empty, `shell: bash`, `set -euo pipefail`, **no `-r`**);
`bash -c 'source /opt/ros/rolling/setup.bash && colcon build --packages-up-to <three> --cmake-args
-DBUILD_TESTING=ON'`; `colcon test --packages-select <three> --event-handlers console_direct+
--return-code-on-test-failure --ctest-args -LE linter` **without sourcing `install/`**; `colcon test-result
--verbose` with `if: always()`. Linters: not run as a class, because they fail today for reasons unrelated to
Rolling (copyright, cpplint, uncrustify, flake8, pep257, `lint_cmake`: see follow-up; #98). One linter is
cheap and catches cause 9 (which `-LE linter` and a package without `ament_lint_auto` both hide): a step
`ament_xmllint marine_nav_interfaces marine_nav_utilities marine_nav_tasks` run from the checkout (it comes
with `ament_lint_common`, a `test_depend`, so rosdep installs it; to be confirmed on the first CI run; if the
container lacks it the step is dropped and the plan falls back to the local check).

**Job `jazzy` on `rolling`** (decision): the nine-package list, `-r`, the `marine_control` clone at `jazzy` and
`-LE linter`/`-m "not linter"` stay exactly as they are (commit 7 changes only the triggers and the job
name). The six Nav2 packages need `-r` today (their keys pull Nav2 and `marine_control`'s dependencies) and it
passes, so touching it is not part of a port. `marine_control` stays at `jazzy`: the job builds a Jazzy
workspace, and the file is identical on both branches, so a branch-dependent ref would make the two copies
differ; `marine_control`'s `rolling` branch builds on Jazzy by its own CI, so switching later is cheap.
Cost of leaving it: the Jazzy job over the whole `src` with `-r` cannot show a key the three packages
need but forgot, nor an unresolvable key, because the broad install pulls in everything first.
**Commit 8 (droppable, my recommendation: keep)** adds a first phase to the Jazzy job: strict `rosdep install`
(no `-r`) over the same `colcon list --packages-up-to <three>` paths, then `colcon build`/`test` of the three
with `-LE linter`, *before* the existing broad `-r` install and nine-package build. That is the only place
the cause 11 declarations are checked on noble. Cost about +1 minute (second `colcon build` of the nine
re-uses the three).

Durations (measured: Jazzy `build-and-test` 302 s on the last `jazzy` push, 195 s of it in the build of nine;
`mru_transform`'s two jobs 2 min 18 s and 2 min 24 s on 2026-11-23): Jazzy job about 5 min as today, about 6
min with commit 8; Rolling job about 2.5 min (container 12 s, tools 33 s, rosdep about 30 s, build of three
about 25 s, tests about 15 s). The two run in parallel, so a pull request to `rolling` waits about 6 min; one to
`jazzy` runs the Jazzy job only.

## State of `rolling` afterwards (for the pull request)

The three packages build and pass their tests on Rolling. The six Nav2 packages
(`marine_nav_behavior_tree`, `marine_nav_behaviors`, `marine_nav_bt_task_navigator`,
`marine_nav_avoidance_controller`, `marine_nav_crabbing_path_follower`, `marine_nav_ca_safety`) still use
`ament_target_dependencies` (counts in #415: 12, 2, -, 2, 7, 2) and more, so a plain whole-repository build on
Rolling fails by design until their own issue. Always build with a package selector.

## Left alone and tracked (one follow-up issue)

- gz4d: the table above (including `transforms.hpp:5`); `angles.hpp` defects to the six-package issue.
- `marine_nav_interfaces`: `CMakeLists.txt` never calls `ament_lint_auto` (its `test_depend`s are dead, no
  xmllint, no tests); `std_msgs` found in CMake but unused by any interface; `cmake_minimum_required(VERSION
  3.8)` (CMake 4 deprecation warning; the other two are at 3.14.4).
- Linters excluded from CI (#98), failing today: `copyright` (all three: headers missing, e.g.
  `corridor_solver.h`, `task.h`), `cpplint`, `uncrustify` (tens of files in `marine_nav_utilities`, four in
  `marine_nav_tasks`), and in `marine_nav_tasks` also `flake8`, `pep257` (`task.py:19` D300) and `lint_cmake`
  (`CMakeLists.txt:39` trailing whitespace). `ament_cppcheck` passes here but skips files on Rolling per #415.
- `marine_nav_tasks` Python has no tests; `task_list.py:47` `if task_id is self.tasks:` (should be `in`: the
  method always returns `None`); `task.py:21`, `task_list.py:24` annotate `rclpy.node.Node` with only
  `import rclpy`. Seen, not touched.
- `marine_nav_utilities/package.xml` declares `ament_cmake_ros` as buildtool but `CMakeLists.txt` uses
  `find_package(ament_cmake)`; `test_costmap_window_node` uses the default DDS domain (no isolation).
- Rolling emits no deprecation warning from these three packages (checked: none).

## Consumers and what must not change

`camp` (`marine_nav_interfaces` `task_information.hpp`, `task_feedback.hpp`; `marine_nav_tasks` `task.h`,
`task_list.h`; `find_package(marine_nav_tasks)` and `ament_target_dependencies(... marine_nav_tasks)`),
`manda_coverage` (`compute_sonar_coverage_path.hpp` action, `utilities.h` via
`marine_nav_utilities::marine_nav_utilities`, `${marine_nav_interfaces_TARGETS}`), `mission_manager`
(`marine_nav_interfaces`, `marine_nav_tasks` Python `Task`/`TaskList`) and `marine_simulation` (messages).
None of those interfaces changes: no message, action, header (apart from the `tf2/utils.hpp` include inside
`utilities.h`, which is exactly what its consumers already need on Rolling), class or signature is edited.
The exported targets (`marine_nav_utilities::marine_nav_utilities`, `marine_nav_tasks::marine_nav_tasks`,
`${marine_nav_interfaces_TARGETS}`) and exported dependencies are the same after commit 1 (installed export
files identical, see commit 1). `rosidl` consumers (`mission_manager_interfaces`) additionally see
`builtin_interfaces` and `std_msgs` declared, which only matches what the generated interface already needed.

## What this code is for, and validation owed

These packages are on the vehicle's mission path, not the helm path: task lists (`Task`/`TaskList`,
`RunTasks`/`Hover` actions) carry the mission from the operator and `mission_manager` to the behavior tree, and
the corridor solver and costmap-window utilities feed path following and obstacle avoidance. A defect here
changes what the boat is told to do.

Behaviour-neutral argument per change: 1, a deletion of calls whose every effect is already in
`target_link_libraries` (link line and exports unchanged); 2, the same function from the same library under its
non-deprecated name (`tf2/utils.hpp` is what `tf2/utils.h` forwarded to on Jazzy, read: the Jazzy `.h` is a
shim that includes the `.hpp`); 3, element order in a manifest; 4, 5, manifest declarations only, no code or
link change; CI, no vehicle code.

Not provided by this port: no simulation or on-water run; no run on Jazzy (this machine is Ubuntu 26.04
with Rolling only, no Docker; Jazzy is checked by reading upstream sources and by the CI Jazzy job; the whole
Jazzy build can be run on deadpool); no test of the Task/TaskList Python (none exist; it only imports cleanly
on Python 3.14, checked); `marine_nav_interfaces` and `marine_nav_tasks` have no tests beyond a successful
build; the 32 gtest cases of `marine_nav_utilities` pass on Rolling (below) but do not exercise `tf2::getYaw`
under any new condition. The Rolling result also says nothing about the six Nav2 packages, Nav2's own Rolling
state, or `camp`, `manda_coverage` and `mission_manager` building against these packages (they are rebuilt in
their own ports).

Flakiness check on the scratch copy (planned fixes applied): `test_corridor_solver` 100/100 and
`test_costmap_window` 100/100 passes as separate processes; `test_costmap_window_node` 300/300, plus 200 runs
8 at a time on one `ROS_DOMAIN_ID` (174; 173 sequentially): 0 failures. No intermittent failure.

## Prior Art

Not new: the fixes are the ones #415 collected from `marine_control`, `marine_ais`, `unh_marine_autonomy`
and `mru_transform`; the workflow copies `mru_transform`'s two-job shape. The strict-rosdep-before-broad-install
phase (commit 8) is new here and is the one element not taken from a model.

## Serves

None (this repository has no written goals or design document); serves the Rolling port, rolker/unh_marine_autonomy#415,
and unblocks #419 (`mission_manager`), `camp`, `manda_coverage`.

## Design Picture

No change to the design picture (no design document; no interface changes).

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | gz4d headers nothing compiles are left alone; follow-ups go to one tracked issue (owner's rule) |
| Verify before claiming | every claim above was built or read; the Jazzy half is labelled as read, not run |
| Test what breaks | no gz4d fix is claimed, because a build cannot exercise one |
| A change includes its consequences | manifests (cause 11) and CI (triggers, Rolling job) ride with the port |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 | Yes | `## Plan Authored` entry in `progress.md` |
| 0005 | Yes | CI is the authoritative check; Rolling job and triggers make it visible on `rolling` |
| 0014 | Yes | phase handoff; no push from this phase |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| cause 1 | consumers' exported targets | Yes: identical, checked |
| cause 2 | `gz4d/transforms.hpp` include | No: not compiled, follow-up |
| `ci.yml` triggers | job names (no required check exists) | Yes |

## Open Questions

- Commit 8 (strict check in the Jazzy job): keep (my recommendation, +1 min, only check of the new
  declarations on noble) or drop (least change)?
- `marine_control` clone ref on `rolling`: `jazzy` (recommended, file identical on both branches).
- `std_msgs` in `marine_nav_interfaces`: declare now (planned), drop from CMake later (follow-up)?
- One more `ament_xmllint` step in the Rolling job: keep (recommended) if the container has the tool.

## Estimated Scope

Single pull request into `rolling`, `Part of #113`; the owner closes the issue after the backport to `jazzy`.
Pull request wording: what was built and tested on Rolling; what was only read for Jazzy; the state of
`rolling` above; what is left and the follow-up issue.
