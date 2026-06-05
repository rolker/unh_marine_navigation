---
issue: 64
---

# Issue #64 — Collision avoidance at survey speed: turn-before-slowdown is speed-limited (near-miss)

## Issue Review
**Status**: complete
**When**: 2026-06-04 19:23 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #64
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/64#issuecomment-4626804034
**Scope verdict**: needs-splitting

### Actions
- [ ] Split #64 into sub-issues (keep #64 as umbrella): (a) speed-scaled slowdown zones [config in seafloor/echoboats]; (b) custom e-stop node (yaw-preserving + reverse-arc) [this repo]; (c) yaw-sign-in-reverse sim verification spike [blocks (b)]; (d) recovery-BT back-off branch; (e) aft reflex cloud [if scope expands]
- [ ] Settle replace-vs-augment with the existing nav2 Collision Monitor (reconcile chain placement vs seafloor #25/#27/#36 + velocity_smoother) before planning the node
- [ ] Default reflex scope to forward-only + aft-gated reverse for June; defer all-around
- [ ] Treat as safety-critical: sim validation + regression tests in-scope, not follow-ups (yaw-sign-in-reverse, reverse closed-loop termination, speed-scaled zone extremes, escape-when-avoider-gave-up)
- [ ] ADR-0008: declare + validate node parameters (finiteness/domain), per #62 precedent
- [ ] Capture the safety-behavior decision (replace CM + autonomous reverse near obstacles) durably; operator-visible/configurable/abortable (CAMP annunciation)
- [ ] Coordinate speed-scaled polygon config with seafloor #3 (model split, in flight); principled fix tracked at seafloor #42
- [ ] Add /odom (mavros local_position/velocity_body) subscription; wire back-off into run_tasks.xml RecoveryNode

## Plan Authored
**Status**: complete
**When**: 2026-06-04 20:08 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-64/plan.md` at `5ebde97`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/68 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] Replace vs augment the nav2 Collision Monitor (plan assumes replace)
- [ ] Stale/absent pointcloud behavior (passthrough / hold-last / fail-safe stop)
- [ ] Confirm cmd_vel message type is TwistStamped against the live chain
- [ ] Capture the safety-behavior decision (PR rationale vs project docs/decisions)

## Plan Review
**Status**: complete
**When**: 2026-06-04 20:51 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review; incorporated an independent fresh-context sub-review)

**Plan**: `.agent/work-plans/issue-64/plan.md` at `675dee1`
**PR**: https://github.com/rolker/unh_marine_navigation/pull/68
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Reflex cloud is in `bizzy/base_link_level` (stabilized, tf_prefixed), not `base_link`; add `tf2_sensor_msgs` dep + name the transform mechanism + handle tf_prefix + level-vs-base distinction — plan.md:31,33,70,84
- [ ] (must-fix) Specify QoS per endpoint (pointcloud best-effort/sensor-data; state+polygons reliable depth1; cmd_vel matched to smoother/echo_helm) — stack has silent QoS-mismatch history (#56) — plan.md:32-48,70-77
- [ ] (must-fix) Drive viz + CollisionMonitorState from an independent ~1-2 Hz timer, not piggybacked on cmd_vel (CAMP 2 s watchdog blanks fills if cmd_vel pauses); state cmd_vel_out behavior when input stops — plan.md:34,46-47
- [ ] (must-fix) Sole-helm-publisher invariant + CM→node cutover ordering as explicit risk (wiring in seafloor#43); note CM-as-passthrough on non-reflex boats — plan.md:19-20,83
- [ ] (must-fix) Make sim/integration validation concrete (scenarios+acceptance or launch_testing); GATE the reverse-yaw-passthrough path off until the yaw-sign-in-reverse sim test passes (issue's stated blocker, currently demoted to a live toggle) — plan.md:38,60-61,106,137
- [ ] (suggestion) Rename package `marine_nav_ca_safety` (executable `ca_safety_node`) per REP-144 — plan.md:24,68,71
- [ ] (suggestion) Justify the `nav2_msgs` coupling (one line) — plan.md:70
- [ ] (suggestion) Specify odom-loss handling in reverse: duration/distance hard backstop independent of odom — plan.md:36-38,49-51
- [ ] (suggestion) Add launch_testing artifact to Files table if adopting the integration-test finding — plan.md:66-77

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-04 22:33 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-64 at `e2e811f`
**Mode**: pre-push
**Depth**: Deep (reason: safety-critical, replaces sole helm publisher)
**Must-fix**: 4 | **Suggestions**: 4

### Findings
- [ ] (must-fix) Reverse backstop resets on transient declassification → runaway into unsensed stern (Claude+Copilot cross-confirmed) — `ca_safety_node.h` doStop/resetReverse
- [ ] (must-fix) Distance backstop permanently disabled if odom stale at reverse start (Claude+Copilot) — `ca_safety_node.h:313`
- [ ] (must-fix) Unvalidated input twist (NaN/inf) propagates to helm (Copilot) — `ca_safety_node.h` cmdVelCallback
- [ ] (must-fix) stop_speed_eps unvalidated, not dynamic (Claude) — `ca_safety_node.h:62`
- [ ] (suggestion) Enforce slowdown min<=max cross-field (Claude+Copilot)
- [ ] (suggestion) source_loss hold uses stale cloud indefinitely — warn (Copilot)
- [ ] (suggestion) Default rclcpp::Time members clock-type fragile — init from clock (Claude)
- [ ] (suggestion) Integration test missing closed-loop stop + flicker coverage (Claude+Copilot)

**Resolved**: all 4 must-fix + 4 suggestions addressed in `7d0a7b9` (rebuilt + retested: 17 pure gtests, 12 node gtests, 4 launch tests, cppcheck + python lint green). Remaining colcon-test lint = repo-convention copyright/cpplint + uncrustify 0.78.1 drift only.
