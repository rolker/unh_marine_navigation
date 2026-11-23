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

## Plan Review
**Status**: complete
**When**: 2026-11-23 12:30 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #113 — Build on ROS 2 Rolling, first slice: marine_nav_interfaces, marine_nav_utilities, marine_nav_tasks
**Plan**: `.agent/work-plans/issue-113/plan.md` at `eb2031b`
**Branch**: `feature/issue-113`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Three packages, five one-fix code commits, two CI commits; matches the owner's slice and the checkpoint's gz4d decision. |
| Issue alignment | Good | Causes 1, 2, 9, 11 and the CI shape answer the issue and the Issue Review's actions; cause 10 follows the host checkpoint (only gz4d headers a Rolling build compiles). Checked: no source or test of the three includes a gz4d header but `gz4d/vector.hpp` (`src/utilities.cpp:2`; the other hits are header-to-header includes), and no consumer clone under `/home/roland/project11/rolling` includes any `marine_nav_utilities/gz4d/` header. Serves: "None (this repository has no written goals or design document); serves the Rolling port, rolker/unh_marine_autonomy#415, and unblocks #419 (`mission_manager`), `camp`, `manda_coverage`." |
| File targeting | Good | All files right. `utilities.cpp` includes `utilities.h` (which pulls `rclcpp`, hence `<utility>`/`<cmath>`) before `vector.hpp`; `vector.hpp` alone fails with GCC 15 (checked), as the plan says. |
| Consequences | Needs work | Exports unchanged by commit 1 (confirmed by reading Jazzy's function: it adds nothing to `INTERFACE_*`). But commit 1's compile-flag claim and the CI's ability to check cause 11 are overstated (findings 1, 2); the one caller of the defective `TaskList.task()` is not named (finding 3). |
| Principle alignment | Good | Only what's needed held; validation statement mostly honest (finding 4). |
| ADR compliance | Good | 0013, 0005, 0014 addressed. |
| ROS conventions | Good | Dependency kinds right: `<depend>` for `builtin_interfaces`/`std_msgs` in a rosidl package and for `geometry_msgs` (named in installed `task.h`); `<exec_depend>` for `rclpy`, `python3-yaml` (runtime-only Python imports). Keeping `std_msgs` in `DEPENDENCIES` and declaring it is sound: dropping it would change the generated package's exported dependencies. |

### Findings

1. **[Consequences]** (low) Commit 1's evidence misdescribes Jazzy's `ament_target_dependencies`. Read at `ament/ament_cmake` branch `jazzy`: besides `target_link_libraries(${target} ${interfaces})` (plain signature, here identical to what is already linked, so link line and exports are unchanged), it also adds every linked target's `INTERFACE_INCLUDE_DIRECTORIES` as `target_include_directories(${target} PRIVATE ...)`, ordered by `ament_include_directories_order`. On Jazzy, deleting the call therefore does change `flags.make`: those directories stop appearing as `-I` and remain only as the targets' `-isystem` usage requirements. Plan section (a)/(b) compares against a stand-in that "adds only what (a) says Jazzy adds", i.e. links only, so "compile flags byte-identical" is not shown for Jazzy. Effect is benign (same headers found, the package's own `include/` stays first; warnings from ROS headers are now suppressed as system headers; exports unchanged because the addition is PRIVATE). Resolution: correct the claim in the plan and the pull request text ("link line and installed exports identical; on Jazzy the call's PRIVATE `-I` copies of dependency include directories go away, which the CI Jazzy job builds"), not a code change.
2. **[Consequences]** (medium) CI cannot check that the cause 11 declarations are needed or complete. REP 2001 `ros_core` (both `ros:jazzy-ros-core` and `ros:rolling-ros-core`) already contains `common_interfaces` (`std_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`), `builtin_interfaces`, `rclpy`, `rclcpp_lifecycle`, `ament_lint_common`; `python3-yaml` comes with `ros2cli`. A strict rosdep without `-r` in either job proves only that each declared key resolves on noble/resolute; a forgotten one among these would still build and pass. Only `tf2`, `tf2_geometry_msgs` and `yaml_cpp_vendor` (outside `ros_core`) are really exercised by the strict install. So "That is the only place the cause 11 declarations are checked on noble" (commit 8) overstates it: commit 8 checks key resolution on noble and a Jazzy build of the three with their own declared closure before the broad `-r` install — real but small value. Resolution: restate commit 8's purpose as that, and say in the validation statement that cause 11 is checked by reading the sources plus key resolution, not by CI failing without it. Keep or drop commit 8 is the owner's open question; either is fine.
3. **[Consequences]** (low) The `task_list.py:47` defect (`if task_id is self.tasks:`) is read correctly: `is` compares a `str` with the dict, always False, so `TaskList.task()` always returns `None`. Its only caller anywhere (this repository, `camp` (C++, no use), `manda_coverage`, `unh_marine_autonomy` `rolling`) is `mission_manager/mission_manager/mission_manager.py:232`, in `navigator_feedback_callback`, inside `if task_feedback.current_navigation_task in self.behavior_library.keys():`. `behavior_library` is never filled (its only writer, `:148`, is commented out), so the branch is unreachable today and the defect is latent, not live. A one-character fix would only matter when that code is re-enabled, and the loop it guards has its own bug (`:236-237` assigns a fresh message's field to itself) and indexes `behavior_info_publishers`, also never filled. Resolution: keep it out of the port, but make the follow-up line name the caller and that the two must be fixed together when `mission_manager`'s behaviour-information feature is revived (better as its own line or issue against `unh_marine_autonomy`'s `mission_manager` than as a gz4d-style note). No need to raise it to the owner as more than that, since it cannot fire today.
4. **[Principle alignment]** (low) Validation statement: add findings 1 and 2, and note that `ament_xmllint` in the Rolling job needs `source /opt/ros/rolling/setup.bash` (it is in `/opt/ros/rolling/bin`, already present in `ros-core` via `ament_lint_common`, so the "if the container lacks it" fallback is moot), and that only `marine_nav_interfaces` (format 3, with the `xml-model` schema line) is schema-checked by it; the two format-2 manifests have no schema reference, so xmllint checks only well-formedness there. Also the follow-up text "these headers have not compiled since the namespace change" should except `vector.hpp` (and `matrix.hpp`), which compile in context but are not self-contained.

### Checks of the host's six points

1. Cause 1 deletion: link line and installed exports unchanged on both distros (confirmed by reading Jazzy's function; Rolling has no such function); compile flags change on Jazzy (finding 1). Each commit builds alone on Jazzy: commit 1 (deletion; `tf2/utils.h` is a shim on Jazzy that includes `tf2/utils.hpp`, read at `ros2/geometry2` `jazzy`), commit 2 (`tf2/utils.hpp` exists on Jazzy), 3-5 manifest only. On Rolling commit 1 alone fails at cause 2, either order; order is fine for cherry-pick.
2. gz4d reading confirmed (grep of the three packages and tests, consumers' clones; `vector.hpp` alone: 7 errors with GCC 15; `utilities.cpp` includes `utilities.h` first). Nothing in this slice changes what precedes `gz4d/angles.hpp` in `crabbing_path_follower.cpp` (the `tf2/utils.h` -> `.hpp` change is include-identical on Jazzy). Leaving `vector.hpp` non-self-contained is within "fix only what the build needs" and is listed in the plan's gz4d table.
3. Cause 11 kinds and keys confirmed; CI can only check resolution (finding 2).
4. CI: shape matches `mru_transform` `rolling` `ci.yml`; Jazzy job on `rolling` still passes after this slice (the six link `marine_nav_utilities::marine_nav_utilities`, whose exports are unchanged); on `jazzy` after backport the Rolling job is skipped by its `if`; rulesets: `jazzy` requires a pull request and no named check, `rolling` has no rules, so renaming the job is safe. Commit 8 sound but overstated (finding 2); `ament_xmllint` sourcing (finding 4).
5. `task_list.py:47` reading confirmed; caller and reachability in finding 3.
6. Validation statement honest on what was not run; incomplete on findings 1, 2 and 4.

### Summary

The plan is sound and ready: the fixes are the minimal ones, every key resolves, and the gz4d reading of the build is right. The findings correct the evidence wording (commit 1 on Jazzy, what CI can prove about cause 11) and sharpen the `TaskList.task()` follow-up; none changes a commit.

### Recommended Actions

- [ ] Reword commit 1's evidence in the plan and pull request: link line and exports identical; on Jazzy the PRIVATE `-I` include directories the call added go away (finding 1).
- [ ] Restate what commit 8 and the Rolling job's strict rosdep prove (key resolution and a narrow-closure build), and say cause 11 completeness rests on reading the sources, since `ros_core` already carries most of those packages (finding 2).
- [ ] In the follow-up, name `mission_manager.py:232` as the only caller of `TaskList.task()`, note it is unreachable today because `behavior_library` is never filled, and that both must be fixed together (finding 3).
- [ ] Run `ament_xmllint` under `source /opt/ros/rolling/setup.bash`; drop the "if the container lacks it" fallback; note that only the format-3 manifest is schema-checked; except `vector.hpp`/`matrix.hpp` from "have not compiled" in the gz4d follow-up (finding 4).

## Checkpoint
**Status**: complete
**When**: 2026-11-23 12:31 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the Plan Review's verdict is ready; its four findings correct what the plan claims and none changes the code to be written)
**After**: plan
**Decision**: proceed

Read: `plan.md` at `eb2031b` and the Plan Review of it (ready; four findings; the six points the host asked about confirmed, two with corrections). The findings go to the implementer as corrections to what is claimed, in commit messages, the entry and the pull request text:

- F1: deleting the `ament_target_dependencies` calls leaves the link line and the installed exports identical; on Jazzy it also removes that function's PRIVATE `-I` copies of the dependencies' include directories, which remain as the targets' usage requirements. "Compile flags byte-identical" was not shown for Jazzy and is not to be claimed; the Jazzy CI job is the check.
- F2: CI cannot show that the cause 11 declarations are needed or complete, because the `ros-core` images already contain those packages. The strict rosdep step checks only that each declared key resolves, and commit 8 (the strict first phase in the Jazzy job) gives a Jazzy build of the three with their own declared closure before the broad install. Cause 11 here rests on reading the sources. Commit 8 is kept, with its purpose stated as that; host decision, listed for the owner's veto.
- F3: `marine_nav_tasks/marine_nav_tasks/task_list.py:47` (`if task_id is self.tasks:`) is read correctly and makes `TaskList.task()` always return `None`; its only caller, `mission_manager.py:232` in rolker/unh_marine_autonomy, is unreachable today. It stays out of the port and goes to the follow-up, naming the caller.
- F4: the `ament_xmllint` step in the Rolling job needs `source /opt/ros/rolling/setup.bash`; it schema-checks only the format 3 manifest (`marine_nav_interfaces`); the follow-up text about gz4d headers excepts `vector.hpp` and `matrix.hpp`, which compile in context but not alone.

## Implementation
**Status**: complete
**When**: 2026-11-23 12:41 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-113 at `ed01bff`

Commits, in backport order (all for the hand backport to `jazzy`; 1 and 2 travel together, since 1 alone does not build on Rolling):

1. `3ab7bbb` cause 1: delete both `ament_target_dependencies` calls. Not a behaviour change: every target the calls linked is already in `target_link_libraries`.
2. `51a9f1e` cause 2: `tf2/utils.h` -> `tf2/utils.hpp` in `utilities.h`, `utilities.cpp`, `costmap_window.cpp`. Not a behaviour change: the same declarations under the non-deprecated name (Jazzy's `.h` is a shim including the `.hpp`).
3. `359f699` cause 9: `test_depend`s before `member_of_group` in `marine_nav_interfaces/package.xml`. Element order only.
4. `77642ef` cause 11: `builtin_interfaces`, `std_msgs` declared in `marine_nav_interfaces`. Manifest only; generated code, libraries and CMake exports unchanged.
5. `e7c0893` cause 11: `geometry_msgs`, `rclpy` (exec), `python3-yaml` (exec) declared in `marine_nav_tasks`. Manifest only.
6. `77de024` CI: triggers `[rolling, jazzy]`, job renamed `jazzy`, new `rolling` job for the three (strict rosdep, no install sourcing, linters excluded, `ament_xmllint` step).
7. `ed01bff` CI: strict first phase for the three in the Jazzy job (key resolution on noble and a Jazzy build and test with their own closure; cannot show cause 11 is needed or complete, F2).

No gz4d change. The build's `.d` files show `gz4d/vector.hpp` (from `src/utilities.cpp`) is the only gz4d header compiled.

Evidence (Rolling only; Jazzy not installed here):
- `./build.sh --packages-up-to` the three, from clean: 3 finished, no compiler warning. `./test.sh --packages-select` the three with linters excluded: `35 tests, 0 errors, 0 failures, 0 skipped`.
- Lint, all tests: `1036 tests, 962 failures, 25 skipped`, the per-linter results identical to the pre-change stand-in; `ament_cpplint` totals (618, 58) identical to the base commit. `ament_xmllint` on the three manifests: base fails on `marine_nav_interfaces` (cause 9), head passes.
- Export comparison against a hand-written stand-in of Jazzy's `ament_target_dependencies` (link and PRIVATE include part): correction to the plan's claim (b). Each `export_<pkg>Export.cmake` differs in `INTERFACE_LINK_LIBRARIES` only: the call had appended a duplicate copy of the same targets, which is now gone (same targets, same order of first appearance). Every other export file is identical. `marine_nav_utilities` link line identical; `marine_nav_tasks` same libraries, different order and rpath order. Include directories same set, different order. `marine_nav_interfaces`: CMake exports, generated sources and libraries identical; only `package.xml` and the manifest-derived index files gain the two keys.
- CI Rolling job rehearsed under `env -i` (rosdep `--simulate`, xmllint, build, test without sourcing install): all exit 0, `35 tests, 0 errors, 0 failures, 0 skipped`.
- Repeats: `test_corridor_solver` 100/100, `test_costmap_window` 100/100, `test_costmap_window_node` 300/300 on `ROS_DOMAIN_ID` 187; no intermittent failure.

Not done: no simulation was run. No Jazzy build: the pull request's CI is the only Jazzy check. `marine_nav_interfaces` and `marine_nav_tasks` have no tests beyond building (the Python only imports on 3.14). F3 (`task_list.py:47`) left as it is.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-23 12:49 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-113 at `7d1a4bd`
**Base**: origin/rolling
**Depth**: Standard (reason: CI workflow change plus build-system and manifest changes in three packages on the mission path; 10 files, +630 -21 of which ~460 are records)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 1 | **Ship**: recommended — no must-fix findings; remaining suggestions can be applied or tracked

Checked: Jazzy `ament_target_dependencies` (ament/ament_cmake `jazzy`) adds only a plain `target_link_libraries` of targets already linked with the plain signature, plus PRIVATE `-I` copies of their include directories, so exported targets and link set are unchanged (duplicates and order only; both libraries SHARED, consumers in this repository, camp and manda_coverage link through the exported targets). `tf2/utils.hpp` exists on geometry2 `jazzy` (`utils.h` is a shim including it). Workflow read step by step against mru_transform's `rolling` ci.yml: same shape, `shell: bash` where bash is used, job `if` correct for push and pull_request. Dependency kinds right (`<depend>` for builtin_interfaces/std_msgs/geometry_msgs, `<exec_depend>` for rclpy/python3-yaml). Re-run here: build of the three from a clean CMake cache, 0 warnings; 32 gtest cases pass, each gtest binary 30/30 (`ROS_DOMAIN_ID` 187); `ament_xmllint` on the three manifests passes; Python modules import. Outside reviewers: Gemini and Codex completed; their colcon-guard finding is a false positive (colcon-package-selection 0.2.10 exits 1 on an unknown or ignored `--packages-up-to` name, checked), the rest dropped as pre-existing, settled or verified.

### Findings
- [ ] (suggestion) plan.md still claims what the implementation entry corrected: compile flags and every export file byte-identical (`:38-41`, `:192-194`, `:259`) and commit 8 as "the only place the cause 11 declarations are checked" (`:146-149`); bring it in line before the PR text is drawn from it — `.agent/work-plans/issue-113/plan.md:38`
- [ ] (suggestion) add to the follow-up issue: `marine_nav_tasks` exports `rclcpp_lifecycle::rclcpp_lifecycle` (and its public `task.h` includes `lifecycle_node.hpp`) but `ament_export_dependencies` omits `rclcpp_lifecycle`, so `find_package(marine_nav_tasks)` consumers (camp) rely on finding it themselves; pre-existing, not from this diff — `marine_nav_tasks/CMakeLists.txt:63`

## Checkpoint
**Status**: complete
**When**: 2026-11-23 12:49 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the pre-push review approved round 1 with no must-fix; Jazzy cannot be built on the development machine and a pull request into `rolling` has had no CI here, so the pushed branch's first run is the first Jazzy evidence and the first test of the workflow change)
**After**: publish
**Decision**: publish

Read: the Local Review (Pre-Push) at `7d1a4bd`, verdict approved, ship recommended, two suggestions open: `plan.md` still claims byte-identical compile flags and export files and overstates what the strict Jazzy phase checks (`.agent/work-plans/issue-113/plan.md:38`); the follow-up issue should gain that `marine_nav_tasks` exports `rclcpp_lifecycle::rclcpp_lifecycle` without `rclcpp_lifecycle` in `ament_export_dependencies` (`marine_nav_tasks/CMakeLists.txt:63`, pre-existing). They stay open and go through triage after publish, so the first CI run starts now. The pull request text is written from the implementation entry's corrected claims, not from the plan's. This is a project issue, so the host runs no full pre-commit pass. The pull request says `Part of #113`.

## Integrated Review
**Status**: complete
**When**: 2026-11-23 12:54 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**PR**: #114 at `97f93b9`
**Sources**: 3 (Local Review (Pre-Push) @ `7d1a4bd` (covers `97f93b9` via bookkeeping), GitHub reviews (none), CI rollup run 45231922301)
**Cross-source confirmations**: 0
**CI**: all-pass

GitHub: no reviews, no inline or conversation comments, no requested reviewers; no Copilot review and no Copilot check-run (the only check-runs are the two `build-and-test` jobs). Gemini and Codex ran in the pre-push round; their findings were weighed there (colcon-guard claim a false positive, the rest pre-existing, settled or verified) and are not re-listed.

CI, first run on a pull request into `rolling` and first run of the changed workflow:
- `build-and-test (rolling)` pass, 1m23s: strict `rosdep install` "All required rosdeps installed successfully"; `ament_xmllint` "File '...package.xml' is valid" for all three, "No problems found"; build "Summary: 3 packages finished [22.1s]"; test-result "Summary: 35 tests, 0 errors, 0 failures, 0 skipped". No compiler warnings.
- `build-and-test (jazzy)` pass, 3m49s. Strict first phase: rosdep without `-r` succeeded, build "Summary: 3 packages finished [21.6s]", test "Summary: 3 packages finished [0.48s]" (`--return-code-on-test-failure`, no separate test-result step). Existing phase: build "Summary: 11 packages finished [2min 0s]" (the nine plus `marine_control_interfaces`, `marine_control`), so the six Nav2 packages compile and link against the changed `marine_nav_utilities` and `marine_nav_tasks` (no `ament_target_dependencies`, `tf2/utils.hpp`); test "Summary: 9 packages finished", test-result "Summary: 262 tests, 0 errors, 0 failures, 0 skipped". No warning or deprecation from the three packages; the compiler warnings shown (unused parameters, sign compare in `crabbing_path_follower.cpp`, `bt_types.cpp`, `set_polygon_from_task.cpp`, `task_navigator.cpp`) are in unchanged files and pre-existing.

### Findings
- [x] (suggestion, Local Review (Pre-Push) @ `7d1a4bd`) Valid, records only: `plan.md` still claims what the implementation entry corrected. `:38-40` says `flags.make` and every installed export file are byte-identical; the implementation entry (progress.md "Export comparison") found `INTERFACE_LINK_LIBRARIES` loses a duplicate copy of the same targets and, on Jazzy, the PRIVATE `-I` copies go away. `:192-194` repeats "installed export files identical"; `:259` "identical, checked". `:146-148` calls commit 8 "the only place the cause 11 declarations are checked on noble", which plan review finding 2 showed overstated (`ros-core` already carries most of those keys; the strict install checks key resolution and a narrow-closure build). Fix: reword those lines to "link set and exported targets unchanged (duplicates dropped); on Jazzy the call's PRIVATE `-I` copies go away; CI Jazzy job built the nine against it" and "commit 8 checks key resolution and a Jazzy build of the three with their own declared closure"; CI run 45231922301 can now be cited as the Jazzy evidence — `.agent/work-plans/issue-113/plan.md`
- [x] (suggestion, Local Review (Pre-Push) @ `7d1a4bd`) Valid, pre-existing, not a change here: `marine_nav_tasks` links `rclcpp_lifecycle::rclcpp_lifecycle` (`CMakeLists.txt:42`) and its public headers `task.h:5`, `task_list.h:6` include `rclcpp_lifecycle/lifecycle_node.hpp`, but `ament_export_dependencies` (`:63-69`) omits `rclcpp_lifecycle`, so `find_package(marine_nav_tasks)` consumers must find it themselves. Fix: add it to the follow-up issue the host drafts (with `task_list.py:47` and the other left-alone items) — `marine_nav_tasks/CMakeLists.txt` (deferred: not this pull request's: pre-existing; recorded for the follow-up issue the host drafts for the owner)

### False positives

## Checkpoint
**Status**: complete
**When**: 2026-11-23 12:55 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (two valid suggestions are open; one is a records correction and the other a note for the follow-up issue; merging past open findings is the owner's answer only)
**After**: findings
**Decision**: address

Read: the Integrated Review of pull request 114 at `97f93b9`: two valid suggestions, no false positives, CI all-pass (run 45231922301: Rolling 3 packages, 35 tests, 0 failures, the manifest check passing; Jazzy strict phase 3 packages, then 11 packages built and 262 tests with 0 failures, so the six Nav2 packages compile and link against the changed libraries). Addressed: (1) `plan.md` is corrected where it still claims byte-identical compile flags and export files and overstates the strict Jazzy phase; records only, no code. (2) the missing `rclcpp_lifecycle` in `marine_nav_tasks`' `ament_export_dependencies` predates this change and is not this pull request's to fix: it is recorded for the follow-up issue the host drafts for the owner, and its box is ticked as deferred there.

## Implementation
**Status**: complete
**When**: 2026-11-23 12:57 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**PR**: #114 at `ed01bff`
**Addressed**: Integrated Review at `97f93b9` (2026-11-23 12:54 -05:00)
**Commits**: `60a8838`, `9e1b4c3`

Records-only pass: no code, test or CI file changed. `ed01bff` is the last non-record commit on the branch and is unchanged by this pass; `git diff --stat ed01bff..HEAD -- . ':!.agent'` prints nothing.

### Actions
- [x] Plan wording corrected, each passage marked "corrected after implementation (2026-11-23)" with what it first said: commit 1 (b) and the consumers paragraph now say link set and exported targets unchanged, `INTERFACE_LINK_LIBRARIES` loses only the duplicate copy (44 -> 23 utilities, 34 -> 24 tasks; same targets, same first-appearance order), other export files identical, link line identical (utilities) / reordered among shared libraries (tasks), Jazzy PRIVATE `-I` copies go away; Jazzy evidence CI run 45231922301 (11 packages, 262 tests, 0 failures). Commit 8 now described as checking key resolution on noble and a Jazzy build of the three from their own declared closure; CI cannot show cause 11 declarations needed or complete (both `ros-core` images already contain them), cause 11 rests on reading sources. Consequences row updated — `.agent/work-plans/issue-113/plan.md` (`60a8838`)
- [x] `rclcpp_lifecycle` missing from `ament_export_dependencies` — `marine_nav_tasks/CMakeLists.txt:63` (deferred: not this pull request's: pre-existing; recorded for the follow-up issue the host drafts for the owner) (`9e1b4c3`)

Quick checks: none run beyond commit hooks; no script, skill or code changed.
