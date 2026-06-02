---
issue: 59
---

# Issue #59 — Move survey-line obstacle avoidance into a controller decorator (off the BT node)

## Plan Authored
**Status**: complete
**When**: 2026-06-02 10:12 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-59/plan.md` at `fd23ac8`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/60 (`[PLAN]` prefix)
**Phases**: single core PR + two follow-up cross-repo config PRs (echoboats, seafloor)

### Open questions
- [x] Host shape → **decorator wrapper** (inner CrabbingPathFollower)
- [x] Old BT node → **delete now** (registration + plugin_lib_names + tests)
- [x] Package name → **marine_nav_avoidance_controller**
- [x] Timing → **all before June 4** (plugin + config-flip), on-water validation before June 15
- [ ] Mid-line re-send (#35): verify new-goal setPlan() propagates through the wrapper (in-code)

## Plan Review
**Status**: complete
**When**: 2026-06-02 10:44 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-59/plan.md` at `162bcf0`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/60
**Verdict**: changes-requested (architecture approved; plan edits needed)

### Findings
- [ ] (must-fix) "Re-host not rewrite" overstates it — costmap sampling/TF/active-range + near-anchor are net-new code vs Costmap2D API — `plan.md` Approach/Scope
- [ ] (must-fix) Config-flip target unverified — bizzyboat.yaml only comments crabbing; `plugin:` is inherited from a base (seafloor nav2_params.base.yaml / ben nav2_params.yaml declare it) — `plan.md` Files/Consequences
- [ ] (must-fix) BT-node deletion touches marine_nav_behavior_tree/CMakeLists.txt (:74 src, :306-316 test) — `plan.md` Files
- [ ] (must-fix) avoid_speed semantics shift (per-segment stamps → whole-controller setSpeedLimit) — decide — `plan.md` Approach
- [ ] (suggestion) Stack: PR1 solver extraction (green alone), PR2 controller+deletion+wiring — `plan.md` Scope
- [ ] (suggestion) Inner controller pluginlib::ClassLoader must be a member outliving the instance (unload crash) — `plan.md` Approach
- [ ] (suggestion) Add explicit acceptance criterion (sim ahead-replan + on-water) tying to #57 desired outcome — `plan.md`
- [ ] (suggestion) Only echoboats needs the flip for June 4; seafloor/ben stay crabbing-direct (opt-in) — `plan.md` Scope
