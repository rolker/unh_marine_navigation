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
