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
