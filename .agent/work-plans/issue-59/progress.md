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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-02 12:29 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (must-fixes addressed; #4 accepted by operator)

**Branch**: feature/issue-59 at `30080d2`
**Mode**: pre-push
**Depth**: Deep (reason: ~2400 LOC across 5 packages, new nav2 controller, lifecycle/concurrency, safety-relevant)
**Must-fix**: 3 (all fixed) | **Suggestions**: 2 (1 deferred, 1 sim-verify)
**Specialists**: Static (cppcheck+cpplint), Governance, Plan-drift, Claude Adversarial (Deep), Copilot Adversarial (Deep)

### Findings
- [x] (must-fix) avoid_speed left clamped if disabled mid-deviation — restore moved outside the avoid_speed>0 gate — `avoidance_controller.cpp:255`
- [x] (must-fix) setSpeedLimit clobbered the override mid-deviation — capture-not-forward while active; restore on new goal/deactivate — `avoidance_controller.cpp:212,200,deactivate`
- [x] (must-fix) 9 lines >100 chars — wrapped via local declare lambda — `avoidance_controller.cpp:129`
- [ ] (deferred — operator's call) PID integrator carries across a back-to-back new line within pid_reset_threshold_ (1 s); accept transient vs add decorator-driven new-goal reset — `crabbing_path_follower.cpp:218`
- [ ] (sim-verify) server tracks nominal goal while inner tracks reshaped path — confirm progress-checker doesn't abort during a large deviation
- Rejected false positives: zero header.stamp (intentional "latest TF" convention), lethal_cost 253 (INSCRIBED, correct), empty-matrix UB (already guarded)

## Integrated Review
**Status**: complete
**When**: 2026-06-02 12:43 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #60 at `1aa628d` (post-merge of origin/jazzy)
**Sources**: 2 (Copilot R1 @ `1c1f537`, Local Review (Pre-Push) @ `1c1f537`)
**Cross-source confirmations**: 1
**CI**: none configured on the repo

### Findings
- [ ] (cross-confirmed: Copilot + Local Review) per-tick `primary_->setPlan(reshaped)` re-publishes the inner plan + resets current_segment_=0 (O(N) scan) every cycle incl. clear water — low impact; optional fix: skip inner setPlan when not avoiding and nominal unchanged — `avoidance_controller.cpp:287`
- [ ] (valid, Copilot) no gtests for the wrapper (delegation, lifecycle, avoid_speed state machine just changed) — add fake-inner + hand-built Costmap2D harness — `marine_nav_avoidance_controller/CMakeLists.txt`
- [ ] (valid-but-intentional, Copilot) obstacle_avoidance_weight default 1.0 diverges from CorridorParams 0.02 — keep (field-validated); add rationale comment — `avoidance_controller.cpp:135`

### Merge conflicts (resolved 1aa628d)
- adjust_path_for_obstacles.{h,cpp}: jazzy #54 teardown fix vs deletion → deletion wins (node gone)
- run_tasks.xml: jazzy #55 weight=1.0 on BT node vs node removal → removal wins (intent carried by echoboats#207)
- kept jazzy #56 costmap-QoS fix (auto-merged)

### False positives
- (none — all 3 Copilot comments are valid or valid-but-intentional)
