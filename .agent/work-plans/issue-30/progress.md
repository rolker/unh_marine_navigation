---
issue: 30
---

# Issue #30 — Decompose tracklines into short, skippable segments (partial coverage-planner port)

## Plan Authored
**Status**: complete
**When**: 2026-05-31 22:50 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-30/plan.md` at `fd32bc9`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/51 (`[PLAN]` prefix)
**Phases**: single

> Note: plan supersedes the issue-body design (segment decomposition → reactive
> Frenet corridor path-adjuster BT node). Issue text needs human reconciliation.

### Open questions
- [ ] max_xte source — per-line task field (survey spec) vs node param vs crab-follower XTE config
- [ ] Confirm survey-line pose frame == map_tide (else TF into costmap frame)
- [ ] Confirm blackboard "node" is spun so a cached costmap subscription fires (else callback group / poll)
- [ ] Inflation-tail handling — g(cost) keys off inscribed/obstacle+sea_surface bands, not 150 m inflation tails

## Plan Review
**Status**: complete
**When**: 2026-05-31 23:27 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (fresh-context sub-agent review, orchestrated by plan author)

**Plan**: `.agent/work-plans/issue-30/plan.md` at `420ada8`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/51
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Insert into the right tree: there are two — `SurveyLine` (run_tasks.xml:389, inner ReactiveSequence:397, the executed one) and `SurveyLineTask` (:487); rename the `SetPathFromTask` output at :410, not :491. Confirm SurveyLineTask isn't a second survey-following site.
- [ ] (must-fix) "BT nodes manifest" is build-generated (CMake `generate_..._nodes_xml`) — not hand-edited; registering in `bt_register_nodes.cpp` regenerates it. Strike that non-task; instead hand-edit `nav2.btproj` palette + the `<TreeNodesModel>` inside run_tasks.xml (:544-822).
- [ ] (must-fix) Pose frame: `TaskInformation.poses` carry the publisher's (CAMP) frame_id, not forced map_tide; default to ALWAYS transform into the costmap frame and short-circuit only when frame_id already equals it.
- [ ] (suggestion) Commit to the cached-subscription design (node IS spun — `set_controller_speed.cpp` proves it); note the callback runs on the bt_navigator executor thread (distinct from the tick thread) so the mutex is load-bearing — use a pointer-swap snapshot, don't hold the lock across the DP.
- [ ] (suggestion) #23 per-pose stamps are NOT load-bearing for CrabbingPathFollower (it times off the robot pose stamp, selects segments geometrically); don't block path synthesis on preserving per-pose stamps for resampled stations.
- [ ] (suggestion) First subscription-bearing BT node in the repo — sound but unprecedented; keep the snapshot pattern tight.

## Implementation
**Status**: complete
**When**: 2026-06-01 00:15 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Commits**: `6a6d7b2` (node + 8 unit tests + CMake/package.xml/register), `a794fcb` (SurveyLine BT wiring + Groot palette)
**Build**: marine_nav_behavior_tree + marine_nav_bt_task_navigator build clean; node in generated manifest; run_tasks.xml + nav2.btproj well-formed.
**Tests**: 8/8 gtest pass (clear→on-line, blob→bounded re-anchored detour, wall→infeasible, lateral-rate limit, offsets/resample helpers).

### Notes
- [ ] Build gotcha: new BT node in a worktree trips the POST_BUILD nodes-xml generator (overridden-package SONAME shadow). Workaround: prepend the package build dir to LD_LIBRARY_PATH. See plan.md Implementation Notes.
- [ ] Not yet field/sim-tuned: w_xte:w_obs, max_xte, max_lateral_rate are defaults pending sim.

## Integrated Review
**Status**: complete
**When**: 2026-06-01 08:41 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #51 at `0659e74`
**Sources**: 2 (Copilot R1 @ `0659e74`, local `## Plan Review` @ `420ada8`)
**Cross-source confirmations**: 0
**CI**: all-pass (copilot-pull-request-reviewer success)

### Findings
- None actionable. Copilot reviewed 10/10 changed files and generated **0 comments** (overview-only review body). The prior `## Plan Review` changes-requested findings are all verified-resolved in the committed implementation:
  - (must-fix, Plan Review) Insert into executed `SurveyLine` tree + rename output — resolved: `run_tasks.xml:410` `{nominal_survey_path}`, node `:420`
  - (must-fix, Plan Review) Hand-edit Groot palette + inline `<TreeNodesModel>` — resolved: `nav2.btproj:12`, `run_tasks.xml:755`
  - (must-fix, Plan Review) Always TF into costmap frame, short-circuit on frame match — resolved: `adjust_path_for_obstacles.cpp:336` (identity when equal), passthrough on TF failure
  - (suggestion, Plan Review) pointer-swap snapshot / no per-pose stamp dependency / tight snapshot pattern — all followed

### Not review findings (outstanding from plan Open Questions — for the human)
- Reconcile the #30 issue body (still describes old segment-decomposition design) — needs human re-title.
- Sim-tune `w_xte:w_obs`, `max_xte`, `max_lateral_rate` (defaults pending).
- Confirm deployed survey-line pose frame vs costmap `map_tide`.

### False positives
- None.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-01 09:04 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-30 at `f5a3c99`
**Mode**: pre-push
**Depth**: Deep (reason: new subscription-bearing BT node, cross-thread concurrency, cross-layer BT wiring)
**Must-fix**: 3 | **Suggestions**: 8
**Sources**: 3 (Claude adversarial, Copilot adversarial, lead/static)

### Findings
- [x] (must-fix, cross-confirmed Claude+Copilot) Subscription callback captures raw `this`; teardown race / potential UAF on tree-reload/shutdown — capture costmap state in a `shared_ptr<State>` by value — `adjust_path_for_obstacles.cpp:281-286` — **FIXED** `c6ba027` (CostmapCache held by shared_ptr, captured by value)
- [x] (must-fix, Copilot verified) Pinned anchor endpoints skip the lethal-cost check (line 97 unreachable when pinned); lethal cell at `active_begin`/`active_end-1` reported as a clear corridor — return `kInf` when zero-col is lethal — `adjust_path_for_obstacles.cpp:92-99` — **FIXED** `e0ae769` (+ LethalAnchorIsInfeasible test)
- [~] (must-fix → reclassified FALSE POSITIVE, static) "New `ament_uncrustify` divergence" was a **local-tooling artifact**, not a real CI regression. On re-check, the local `ament_uncrustify` (uncrustify 0.78.1) flags *every committed header* in the package with namespace-indentation + `std::vector < double >` template spacing — neither of which the committed, CI-merged codebase uses. If CI enforced the local tool's output no file would pass. The `.cpp` flag was lambda-body indentation, but this file is the first in the package to use auto-assigned lambdas (no committed precedent), entangled with the same unreliable tool. The file already matches the committed ament style; `--reformat` would mangle it inconsistent with the whole package. Not applied. See review note below.
- [ ] (suggestion, 3 sources: me+Claude+Copilot) Temporal term keyed by station ordinal while path re-resampled each tick; dormant at `w_temporal=0.0` — reproject by arclength / invalidate on path change before enabling — `adjust_path_for_obstacles.cpp:80,100-104,435`
- [ ] (suggestion, Copilot) Defensive costmap-metadata validation (`resolution>0`, `data.size()==size_x*size_y`) before sampling — `adjust_path_for_obstacles.cpp:349-361`
- [ ] (suggestion, Claude) Sampling ignores `origin.orientation`; add identity-orientation guard (nav2 costmap is axis-aligned today) — `adjust_path_for_obstacles.cpp:352-355`
- [ ] (suggestion, Claude) `getCurrentPose` failure starts active range at index 0 (possibly behind boat); prefer passthrough/clamp + throttled log — `adjust_path_for_obstacles.cpp:371-387`
- [ ] (suggestion, Claude) Only first contiguous in-window run optimised; document the limitation — `adjust_path_for_obstacles.cpp:400-406`
- [ ] (suggestion, Claude) `prev` ternary copies the full vector every tick when temporal on; use if/pointer — `adjust_path_for_obstacles.cpp:423-424`
- [ ] (suggestion, Claude+me) OOB sentinel uses exact double-equality on `-1.0`; latent trap if `sample()` ever does arithmetic — consider `std::optional<double>` — `adjust_path_for_obstacles.cpp:36,392,402,419`
- [ ] (suggestion, Copilot) `std::isfinite` validation on numeric BT ports — low priority (static XML literals under our control) — `adjust_path_for_obstacles.cpp:309-317`

### False positives
- (static) cpplint `legal/copyright` + `build/header_guard` on the new files — sibling `clear_path.h` produces the identical 5 hits; pre-existing package-wide convention, not introduced here. The new file correctly matches existing style.
- (static) `ament_uncrustify` "divergence" on the new files — local uncrustify 0.78.1 produces non-ament output (indents namespace bodies, spaces templates) and flags all committed package files; it disagrees with the CI-merged ament style. Local-tooling artifact, not a code defect. Open item for the human: confirm the project repo's CI uncrustify is green on jazzy (it must be, since the package is merged) and, if desired, file a separate issue to reconcile local uncrustify version/config with CI.

## Review Fixes Applied
**Status**: complete
**When**: 2026-06-01 09:30 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Commits**: `e0ae769` (lethal-anchor reject + test), `c6ba027` (callback-lifetime UAF)
**Build/Test**: marine_nav_behavior_tree builds clean; 9/9 gtest pass (8 prior + new LethalAnchorIsInfeasible).
**Must-fix #3 (uncrustify)**: not applied — reclassified as a local-tooling false positive (see Local Review False positives). Surface to human before merge if CI uncrustify behaviour is in doubt.
**Suggestions (8)**: deferred — none are live defects (temporal term dormant at `w_temporal=0.0`; the rest are defensive guards / docs). Candidates for a follow-up hardening pass.

## CAMP Visualization + Hardening
**Status**: complete
**When**: 2026-06-01 10:05 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Commit**: `fcd02af` (CAMP obstacle-avoidance MarkerArray + 3 robustness guards)
**Build/Test**: marine_nav_behavior_tree clean; 9/9 gtest pass; no warnings from the file.

### Operator feedback (the headline)
Investigated what CAMP shows about obstacle reaction (see below), then added a
`visualization_msgs/MarkerArray` publisher on `survey_obstacle_avoidance`,
published only while deviating: faint nominal line + bright adjusted path + red
avoiding band + "AVOIDING" text flag; DELETEALL clears it on avoiding→clear.
CAMP's `MarkersManager` auto-discovers any MarkerArray (markers_manager.cpp), so
no CAMP-side change is needed. Design chosen by Roland (path + avoiding highlight).

**Pre-existing visualization findings (for the human):**
- CrabbingPathFollower already publishes `received_global_plan` (Path) +
  `path_follower_visualization` (MarkerArray, auto-shown in CAMP) — `crabbing_path_follower.cpp:170,176`.
- **Topic-name bug**: CAMP subscribes to `received_global_**path**` (`platform.cpp:103`)
  but the follower publishes `received_global_**plan**` — nothing publishes
  `received_global_path` (no remap found). CAMP's dedicated path renderer is dead;
  the marker layer compensates. Worth a separate fix (CAMP or a remap).
- The local costmap IS visible in CAMP: nav2 publishes `local_costmap/costmap`
  (OccupancyGrid) and CAMP's GridManager auto-discovers any OccupancyGrid by type
  with no namespace filter (`grid_manager.cpp:30`). Operator toggles the grid layer.

### Hardening (review suggestions #5/#6/#7/#9 — the "cheap" set Roland approved)
- [x] (#5) Costmap-metadata validation before sampling (resolution>0, non-empty, data==size_x*size_y) — passthrough on malformed.
- [x] (#6) Identity origin-orientation guard (axis-aligned assumption) — passthrough on rotated/relayed grid.
- [x] (#7) Pose-lookup failure → passthrough (don't reshape from index 0 / astern) + throttled warn.
- [x] (#9) Temporal-term `prev` ref bound to a shared empty lvalue — drops a per-tick vector copy.

### Still deferred (not done)
- (#4) Temporal-term arclength reprojection — dormant at `w_temporal=0.0`; do before enabling.
- (#8) Document the single-contiguous-in-window-run limitation.
- (#10) OOB sentinel → `std::optional<double>` refactor.
- (#11) `std::isfinite` validation on numeric BT ports — low (static XML literals).

## Integrated Review
**Status**: complete
**When**: 2026-06-01 11:12 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #51 at `14ea99c`
**Sources**: 4 (Copilot R1 @ `0659e74`, R2 @ `402edff`, R3 @ `14ea99c`, local Local Review (Pre-Push) @ `f5a3c99`)
**Cross-source confirmations**: 3
**CI**: all-pass (copilot-pull-request-reviewer success)

### Findings
- [x] (cross-confirmed Copilot R2 + Local Review) Pinned anchors skip lethal check — `adjust_path_for_obstacles.cpp:109` — already FIXED `e0ae769`
- [x] (cross-confirmed Copilot R2 + Local Review) Callback captures raw `this` → teardown UAF — `adjust_path_for_obstacles.cpp:385` — already FIXED `c6ba027`
- [x] (cross-confirmed Copilot R2 + Local Review) Ternary prvalue copies prev_offsets_ per tick — `adjust_path_for_obstacles.cpp:577` — already FIXED `fcd02af`
- [x] (valid, Copilot R2+R3) Test missing `<algorithm>`/`<string>` — FIXED `996ecd8`
- [x] (suggestion, Copilot R3) resampleStations O(stations×segments) → single-pass O(segments+stations) — FIXED `996ecd8`

### False positives
- None — all Copilot findings legitimate (3 already fixed, 2 open).

## Review Follow-ups + Optional Slowdown
**Status**: complete
**When**: 2026-06-01 11:40 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Commits**: `996ecd8` (Copilot review: resample single-pass + test includes), `90380bd` (optional avoidance slowdown)
**Build/Test**: marine_nav_behavior_tree clean; 11/11 gtest pass (9 prior + 2 ApplyAvoidanceSlowdown); XML well-formed.

### Copilot review (#4, #6) — done
- #4 test `<algorithm>`/`<string>` includes; #6 resampleStations single-pass walk.

### Optional slow-down through avoidance (#3 — Roland-requested, beyond original scope)
- New `avoid_speed` port (default 0.0 = off). `applyAvoidanceSlowdown()` stamps the
  deviating run so CrabbingPathFollower commands `avoid_speed` there (it derives
  per-segment speed from distance/Δstamp — `crabbing_path_follower.cpp:347-353`).
  **Corrects** the earlier "per-pose stamps not load-bearing" read — the follower
  DOES use them for speed (geometric segment *selection* was the only part that
  was stamp-independent). Port wired into TreeNodesModel + nav2.btproj; default-off
  so deployments opt in.

### Still deferred (unchanged): suggestions #4(temporal)/#8/#10/#11 from Local Review (Pre-Push).

## avoid_speed as a live ROS parameter
**Status**: complete
**When**: 2026-06-01 12:10 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Commit**: `190e414`
**Build/Test**: marine_nav_behavior_tree clean; 11/11 gtest pass; XML well-formed.

- `avoid_speed` BT port now seeds a dynamic ROS param `survey_avoidance_speed` on
  the bt_navigator node (declared once, has_parameter-guarded; descriptor for
  rqt_reconfigure). Read each tick → tunable live:
  `ros2 param set <bt_navigator> survey_avoidance_speed <m/s>`. Port = startup
  seed; param authoritative thereafter (yaml override honoured). Aids sim/field tuning.
- Same pattern could expose the corridor weights (w_xte/w_obs/max_xte/…) as live
  params for the pending sim-tune — offered to Roland, not yet done.
