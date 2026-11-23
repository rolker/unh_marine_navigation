---
issue: 113
---

# Issue #113 — Build on ROS 2 Rolling, first slice: marine_nav_interfaces, marine_nav_utilities, marine_nav_tasks

## Issue Review
**Status**: complete
**When**: 2026-11-23 12:17 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**Issue**: #113

### Scope Assessment

**Well-scoped?** Yes — three packages, a short list of known fixes, and a stated rule for what is left alone. One PR is realistic, with one fix per commit. The three-package cut holds (checked below). The cause list is incomplete in ways that do not change scope (see Recommendations).
**Right repo?** Yes — every change is inside `unh_marine_navigation`.
**Serves**: none — this repository has no written goals or design document; the work serves the Rolling port tracked in rolker/unh_marine_autonomy#415 and unblocks rolker/unh_marine_autonomy#419 (`mission_manager`), `camp` and `manda_coverage`.
**Design picture?** No change — no project design document exists, and no interface, topic or message changes.
**Dependencies**: rolker/unh_marine_autonomy#415 (cause list), #418/#420 (gz4d copy), #419 waits on this. The six Nav2 packages' own issue is independent.

**Checked against the source at `60e4063` (text search vs. source):**

- **Nav2-free: confirmed.** No `nav2_*`, `navigation2`, BehaviorTree or `marine_control` appears in any `package.xml`, `CMakeLists.txt` or source of the three packages (the only hits are comments and a test name saying the QoS matches Nav2's costmap publisher). `marine_nav_utilities/src/costmap_window.cpp` includes only its own header, `tf2/utils.h` and standard headers; `costmap_window.h` includes only `nav_msgs/msg/occupancy_grid.hpp`. It works on a plain `OccupancyGrid`. `marine_control` is needed only by `marine_nav_avoidance_controller`, `marine_nav_crabbing_path_follower` and `marine_nav_ca_safety`, so a three-package job does not need that clone.
- **Cause 1** confirmed: `marine_nav_utilities/CMakeLists.txt:70` and `marine_nav_tasks/CMakeLists.txt:61` are the only calls. Every dependency named is already in the target's `target_link_libraries`, so the calls can be deleted.
- **Cause 2**: the list of removed `.h` headers is complete: `tf2/utils.h` at `utilities.h:8`, `src/utilities.cpp:3`, `src/costmap_window.cpp:6`, and `tf2/LinearMath/Quaternion.h` at `gz4d/transforms.hpp:5`. No other tf2 `.h` in the three packages. `/opt/ros/rolling` has only the `.hpp` names. That Jazzy has `tf2/utils.hpp` and `tf2/LinearMath/Quaternion.hpp` is not checkable here; CI on Jazzy checks it.
- **Cause 9** confirmed: `marine_nav_interfaces/package.xml` has `<test_depend>` after `<member_of_group>`. Note that CI runs `-LE linter`, so the xmllint test that finds this is not run in CI on either distro; it shows only locally.
- **Cause 10 is partly wrong.** `gz4d/geo.hpp:291` (`getMin`/`getMax` on `Bounds`) and `:336` (`std::shared_ptr` without `<memory>`) are the same two defects as #418. But no source of the three packages includes `geo.hpp`, `gz4d.hpp`, `transforms.hpp`, `wgs84.hpp` or `ellipsoid.hpp`; the only gz4d header compiled is `vector.hpp` (from `utilities.cpp:2`), and nothing outside this repository includes any gz4d header from this package (`camp` and `manda_coverage` have their own copies or none). So these headers are installed but never compiled by the build, and applying the two fixes cannot be shown to work by building. Compiled alone with GCC 15 (`g++ -std=c++17 -fsyntax-only`), the copy has more defects than the two: missing standard includes (`angles.hpp` needs `<sstream>`/`<stdexcept>`, `vector.hpp` `<utility>`/`<cmath>`, `matrix.hpp` `<limits>`, `utilities.hpp` `<istream>`/`<iomanip>`), `ellipsoid.hpp:20` redeclares a template parameter name (an error), and `transforms.hpp` uses `Angle` unqualified inside the namespace. So the copy is not "exactly two defects"; it is not self-contained. It is likely these headers have not compiled since the namespace change.
- **Cause 11 candidates** (`package.xml` keys used but not declared, checkable now): `marine_nav_interfaces` uses `builtin_interfaces/Duration` (`action/Hover.action:23`) and lists `builtin_interfaces`, `std_msgs` in CMake but declares neither; `marine_nav_tasks` Python imports `rclpy` and `yaml` (`marine_nav_tasks/task.py`, `task_list.py`) and declares neither `rclpy` nor `python3-yaml`, and its C++ names `geometry_msgs` types directly. `marine_nav_utilities` has no Boost or other non-ROS dependency. `marine_nav_tasks` depends outside ROS only on yaml-cpp (via `yaml_cpp_vendor`, present on Rolling).
- **Causes 4, 5, 6, 7, 8, 13**: none apply (no `setup.py`, no `.warn(`, no Nav2 or geodesy). **Cause 14**: none: no `tf2_ros` use anywhere in the three packages and no `tf2_ros` dependency.
- **Tests**: `marine_nav_utilities` has three gtest files (32 `TEST` cases: corridor solver 15, costmap window 11, costmap window node 6; the node test needs rclcpp discovery and is the one to watch for timing on Rolling). `marine_nav_interfaces` and `marine_nav_tasks` have none beyond `ament_lint_auto`; `marine_nav_tasks`'s Python has no tests.
- **Consumers** (read-only clones under `/home/roland/project11/rolling`): `camp` includes `marine_nav_interfaces` `task_information.hpp`, `task_feedback.hpp` and `marine_nav_tasks/task.h`, `task_list.h` (`running_tasks_*`, and `marine_nav_tasks` in its CMake); `manda_coverage` includes the `compute_sonar_coverage_path.hpp` action and `marine_nav_utilities/utilities.h` (`SurveyPath.cpp`); `mission_manager` and `marine_simulation` use the messages from Python. So `utilities.h` (and its `tf2/utils` include) and the `task.h`/`task_list.h` headers and `Task`/`TaskList` Python API must keep their names and signatures. `marine_nav_crabbing_path_follower` (one of the six) includes `gz4d/angles.hpp`, so that header must stay.
- **Existing CI** (`.github/workflows/ci.yml`): triggers `push`/`pull_request` on `jazzy` only; one job on `ros:jazzy-ros-core`; clones `rolker/marine_control` at branch `jazzy` (shallow) as a source sibling; `rosdep install --from-paths src --ignore-src -r -y` over the whole `src`, so the `-r` hides any missing key (cause 11) and also pulls Nav2 and `marine_control`'s dependencies for the six packages; `colcon build --packages-up-to` the nine packages with `-DBUILD_TESTING=ON`; `colcon test --packages-select` the same nine with `-LE linter` and `-m "not linter"`, so lint tests (copyright, xmllint, cpplint) never run. A pull request into `rolling` gets no CI. No other repository is cloned. The model workflows (`unh_marine_autonomy`, `mru_transform` on `rolling`) carry one file on both branches: a Jazzy job and a Rolling job on `ros:rolling-ros-core` with `if: (github.base_ref || github.ref_name) != 'jazzy'`, and rosdep without `-r` over `colcon list --paths-only --packages-up-to <list>`.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Small steps; step back when they stop converging | OK | Three packages, one fix per commit, left-alone findings go to follow-up issues (owner's rule, quoted in the issue). |
| Test what breaks | Watch | The gz4d fixes (cause 10, and the `transforms.hpp` include) are in headers nothing compiles, so a green build says nothing about them. Say so in the PR rather than count them as checked. |
| Verify before claiming | Watch | The issue's text states the gz4d copy needs "the same two fixes" as the starting point; the source shows more defects (above). The PR should state what was and was not compiled, per "Done when". |
| A change includes its consequences | Watch | Missing `package.xml` keys (cause 11) and the CI triggers/Rolling job are consequences of the port; both are in the issue. The `-r` and the `-LE linter` exclusion hide what the new Rolling job is meant to find. |
| Leave in a project only what it chose to carry | OK | The owner decided that `.agent/work-plans/issue-113/` records ride in the PR. |
| Only what's needed | OK | The cut to Nav2-free packages is the owner's decision and holds on the source. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0013 (progress.md entries) | Yes | This review is written as an Issue Review entry. |
| 0005 (layered enforcement) | Yes | CI is authoritative here; the Rolling job and its triggers are what make the Rolling result visible, and a jazzy-only trigger leaves PRs into `rolling` unchecked. |
| 0014 (phase handoff) | Yes | The issue runs through the dispatched loop; no change to it. |
| Others | No | No workspace infrastructure, adapter or design change. |

### Consequences

- Consumers (`camp`, `manda_coverage`, `mission_manager`) are not rebuilt by this issue. They rely on the interface names above; none changes.
- The `rolling` branch's README or the PR must say the six Nav2 packages do not build on Rolling (already in the issue).
- The Jazzy job's lists and `-r` are a plan decision: the issue leaves it to the plan; the facts above are the starting point.

### Recommendations

- Correct the issue's reading of cause 10 in the plan: `geo.hpp` is compiled by nothing in the three packages and by no outside consumer, and the gz4d copy has defects beyond the two (listed above). Keep the two #418 fixes and the `transforms.hpp` `.hpp` include as the issue says (small, build on GCC 13 and 15), mark them as not exercised by the build, and send the other defects to the follow-up issue next to #420 rather than fixing them here. This does not change scope.
- Add to the cause 11 check, now known from the source: declare `builtin_interfaces` (and `std_msgs` or drop it from CMake) in `marine_nav_interfaces`, and `rclpy`, `python3-yaml` and `geometry_msgs` in `marine_nav_tasks`; run the Rolling job's `rosdep install` without `-r` so CI confirms it. This is within the issue's scope (cause 11).
- Plan the CI as one file carried on both branches, like the model workflows: triggers `[jazzy, rolling]`, a Rolling job for the three packages with `rosdep install` (no `-r`) over `colcon list --paths-only --packages-up-to` the three, no `marine_control` clone in that job, skipped when the base is `jazzy`. Decide in the plan whether the existing Jazzy job keeps its nine-package list and `-r` unchanged (least change; it passes today) and say that the `-LE linter` exclusion stays, so cause 9 is checked by hand locally with `colcon test` on the interfaces package. This is for the plan, not a scope change.
- State in the PR that the six Nav2 packages' `ament_target_dependencies` calls (counts in #415) are left for their own issue, and that on `rolling` the Jazzy-style whole-repository build fails by design.
- Keep `Task`/`TaskList`, `utilities.h` and the message and action names unchanged; they are what `camp`, `manda_coverage` and `mission_manager` use. Not a scope change.

### Actions
- [ ] Correct the issue's reading of cause 10 in the plan: `geo.hpp` is compiled by nothing in the three packages and by no outside consumer, and the gz4d copy has defects beyond the two (listed above). Keep the two #418 fixes and the `transforms.hpp` `.hpp` include as the issue says (small, build on GCC 13 and 15), mark them as not exercised by the build, and send the other defects to the follow-up issue next to #420 rather than fixing them here. This does not change scope.
- [ ] Add to the cause 11 check, now known from the source: declare `builtin_interfaces` (and `std_msgs` or drop it from CMake) in `marine_nav_interfaces`, and `rclpy`, `python3-yaml` and `geometry_msgs` in `marine_nav_tasks`; run the Rolling job's `rosdep install` without `-r` so CI confirms it. This is within the issue's scope (cause 11).
- [ ] Plan the CI as one file carried on both branches, like the model workflows: triggers `[jazzy, rolling]`, a Rolling job for the three packages with `rosdep install` (no `-r`) over `colcon list --paths-only --packages-up-to` the three, no `marine_control` clone in that job, skipped when the base is `jazzy`. Decide in the plan whether the existing Jazzy job keeps its nine-package list and `-r` unchanged (least change; it passes today) and say that the `-LE linter` exclusion stays, so cause 9 is checked by hand locally with `colcon test` on the interfaces package. This is for the plan, not a scope change.
- [ ] State in the PR that the six Nav2 packages' `ament_target_dependencies` calls (counts in #415) are left for their own issue, and that on `rolling` the Jazzy-style whole-repository build fails by design.
- [ ] Keep `Task`/`TaskList`, `utilities.h` and the message and action names unchanged; they are what `camp`, `manda_coverage` and `mission_manager` use. Not a scope change.

## Checkpoint
**Status**: complete
**When**: 2026-11-23 12:17 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the Issue Review found the issue well-scoped and the three-package cut sound on the source; its actions correct the cause list and none changes scope)
**After**: issue-actions
**Decision**: proceed

Read: the Issue Review of 113 (well-scoped, right repository, no design change; none of the three packages needs Nav2 or any of the six packages left out). Its actions go to the planner as inputs:

- Cause 1: both `ament_target_dependencies` calls can simply be deleted, since every dependency they name is already in `target_link_libraries`; confirm by building.
- Cause 11: `marine_nav_interfaces` uses `builtin_interfaces` and lists `std_msgs` in CMake without declaring them; `marine_nav_tasks` imports `rclpy` and `yaml` and names `geometry_msgs` types without declaring them. Each confirmed before it is declared.
- CI: one workflow file on both branches, triggers `[jazzy, rolling]`, a Rolling job for the three packages with `rosdep install` without `-r` over their build closure and no `marine_control` clone, skipped when the base is `jazzy`. The plan says whether the existing Jazzy job keeps its nine-package list and `-r`; the `-LE linter` exclusion means cause 9 is checked by hand.
- Consumers' interfaces (`Task`/`TaskList`, `utilities.h`, the message and action names) stay unchanged; `gz4d/angles.hpp` must stay, as one of the six later packages includes it.
- Host decision on cause 10, listed for the owner's veto: the review found that nothing in the three packages compiles `gz4d/geo.hpp` (only `gz4d/vector.hpp` is compiled, through `utilities.cpp`), that no outside consumer includes this package's gz4d headers, and that the copy has more defects than the two fixed in rolker/unh_marine_autonomy#418. The issue text says to apply those two fixes; that line came from a text search. By the owner's rule that a port changes only what it needs, the plan fixes what a Rolling build of these three packages needs and establishes by building which gz4d headers that is; headers nothing compiles are left as they are and listed, with their defects, for a follow-up beside rolker/unh_marine_autonomy#420. Undoing it: two small edits to `geo.hpp` that no build would exercise.

## Plan Authored
**Status**: complete
**When**: 2026-11-23 12:27 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Plan**: `.agent/work-plans/issue-113/plan.md` at `eb2031b`

Four fix commits (causes 1, 2, 9, 11 in two parts), none for gz4d (only vector.hpp is compiled and GCC 15 accepts it), then a Rolling CI job for the three packages and an optional strict manifest check in the Jazzy job. Built and tested on a scratch copy: 35 tests pass, gtests repeated 100 to 300 times with no failure.
