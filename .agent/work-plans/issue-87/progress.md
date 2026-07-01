---
issue: 87
---

# Issue #87 — Add turn-speed regulation to CrabbingPathFollower

## Issue Review
**Status**: complete
**When**: 2026-07-01 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #87
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Default off/neutral; live-settable via `marine_control`; finiteness-guarded. Issue is explicit on all three. |
| A change includes its consequences | Watch | Unit tests in acceptance criteria; plan phase should also confirm README/doc-comment coverage for new params. |
| Only what's needed | OK | Focused fix for a concrete, measured problem with field evidence. Single file + params + tests. |
| Improve incrementally | OK | Single PR scope; backward-compatible default. |
| Test what breaks | Watch | AC includes unit tests; key edge cases to require: crab_angle=0 (no-op), max angle (max regulation), NaN/Inf/negative inputs, default-off passthrough. Sim test in AC is broad — plan should concretize the scenario. |
| Safety First (project) | OK | Turn slowing is a net safety improvement; default neutral prevents regression. |
| Simulation-First Validation (project) | Watch | AC mentions sim test ("boustrophedon or zig-zag") but no specific sim package/scenario pinned — plan should resolve this. |
| Hardware Agnosticism (project) | OK | Regulation factor is speed-signal-based; no platform-specific path. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 — Worktree isolation | Yes | Already in worktree ✓ |
| 0008 — ROS 2 conventions | Yes | Param declaration, callback, and range descriptor must follow existing patterns (`gainScheduleScale`, `setSpeedLimit` are cited precedents). |
| 0001 — Adopt ADRs | Recommendation | The choice between curvature-based vs. crab-angle-based regulation is a non-obvious design decision. Record the rationale (ADR or detailed plan comment) so future contributors don't accidentally revert or misapply the approach. |

### Consequences

- New params live in `marine_nav_crabbing_path_follower`; must also be bound in the `marine_control` server (same pattern as `default_speed`, `gain_ref_speed`) — verify no marine_control-side changes are needed beyond what `setSpeedLimit`/`declare_parameter` already provides.
- If curvature-based approach is chosen, path curvature computation must handle degenerate inputs: path with fewer than 2 lookahead points, collinear points (infinite radius), NaN waypoints.
- Speed precedence chain (#32) must stay coherent — regulation is a cap applied after the precedence winner, not a new speed source. The plan must show where in the precedence chain the regulation step sits (the issue spec says "after precedence winner, mirroring `speed_limit_`").

### Actions
- [ ] Pin a concrete sim scenario in the plan (package, launch, expected behavior) so the sim acceptance criterion is verifiable, not aspirational.
- [ ] Decide and record (plan or ADR note) which regulation signal is used (curvature vs. crab-angle magnitude) and why — the "evaluate both" framing in the issue is exploratory; the plan phase should close on one approach.
- [ ] Confirm that marine_control integration requires no additional binding changes (or add them to scope if it does).
- [ ] Ensure unit tests cover degenerate inputs: NaN/Inf/negative regulation factor, zero crab_angle (no-op path), default-params passthrough.

## Plan Authored
**Status**: complete
**When**: 2026-07-01 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-87/plan.md` at `d27982b`
**Branch**: feature/issue-87 at `d27982b`
**Phases**: single

### Open questions
- [ ] Sim acceptance criterion: no specific gazebo launch / scenario is pinned in the package — validate via topic logging on a zig-zag path or add a dedicated sim scenario before merge.
- [ ] Signal choice resolved: crab-angle magnitude (over path curvature) — rationale recorded in plan §Context.

## Plan Review
**Status**: complete
**When**: 2026-07-01 01:56 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-87/plan.md` at `d27982b`
**PR**: PR-less (`--issue` mode; `gh` unauthenticated in this worktree — issue/review-issue context read from the persisted `## Issue Review` entry)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Adding two `kTunables` entries grows the advertised control set 10→12, breaking existing `test_crabbing_control.cpp` size assertions (`ASSERT_EQ(..., 10u)` at test lines 119 & 243, plus "ten controls" doc comment L10 and CMake comment L89). Add `test/test_crabbing_control.cpp` to scope (bump to `12u`; assert the two new controls' group/units/range). Plan omits this file and its Consequences "no marine_control-side changes needed" claim is incomplete — `plan.md:87-95`, `plan.md:119`.
- [ ] (suggestion) Pin the `turnSpeedFactor` insertion point to immediately before line 879 (`cos_crab`), i.e. AFTER the trajectory-speed rederivation block at cpp lines 871-877 (`target_speed = segment_distance/dt.seconds();`). The prose "after `gainScheduleScale`" spans that block; inserting at ~L821 silently clobbers regulation on timestamped trajectories. Also correct "same slot as `speed_limit_` cap" — `speed_limit_` is applied at ~cpp L687 and is itself overwritten by the trajectory block — `plan.md:60-70`, `plan.md:120`.
- [ ] (suggestion) `min_factor` upper-bound check (`v > 1.0`) must run BEFORE `update()` stores the atomic, else an out-of-range value is committed before rejection returns — `plan.md:54-58`, `plan.md:122`.
- [ ] (suggestion) review-issue action "pin a concrete sim scenario" remains open (`plan.md:124-126`); either pin the scenario or explicitly waive it in favor of topic-logging validation to close the "Simulation-First Validation" watch.

### Notes
- Verified against source: `gainScheduleScale` (`path_geometry.hpp:130`), `kTunables[]` (`crabbing_path_follower.cpp:44`), param callback + `update()` lambda (`:293`), `computeVelocityCommands` regulation site (`:819-880`), test/CMake patterns. The plan's function/param/test approach matches existing patterns precisely; only the must-fix consequence gap and the two placement/ordering clarifications need addressing before implementation.
- Independent review (plan authored by a Sonnet-model agent; this review by a fresh-context Opus sub-agent).

## Implementation
**Status**: complete
**When**: 2026-07-01 02:15 +00:00
**By**: Claude Opus

**Branch**: feature/issue-87
**Commits**: `67f30ca` (feature + tests), plan/progress sync follows

### What was implemented (plan.md steps 1–8)
- **`turnSpeedFactor()`** added to `path_geometry.hpp` — pure inline function,
  `clamp(1 - |crab_angle_deg| / max_crab_deg, min_factor, 1.0)`; disabled
  (returns 1.0) when `max_crab_deg <= 0`; NaN/Inf crab → `min_factor`.
- **Two atomics** in `crabbing_path_follower.h`: `turn_speed_max_crab_deg_`
  (0.0, disables) and `turn_speed_min_factor_` (0.3), with doc comment.
- **Two `kTunables[]` entries** (`turn_speed_max_crab_deg` deg/[0,90], group
  `speed`; `turn_speed_min_factor` dimensionless/[0,1], group `speed`) — picked
  up automatically by `declareCrabbingControlParams`/`bindCrabbingControls`.
- **Two `read_validated` calls** in `configure()` (both `lo=0.0, exclusive_lo=false`).
- **Two callback branches** in the on-set-parameters callback.
- **Applied `turnSpeedFactor`** in `computeVelocityCommands` (see placement below),
  with a DEBUG log of the regulation factor + regulated speed.
- **`test/test_turn_speed_factor.cpp`** (7 cases) + wired in `CMakeLists.txt`.

### Plan Review findings folded in (all four)
- **[must-fix] `test_crabbing_control.cpp` updated.** Both size assertions
  `10u → 12u` (advertise + wrapped-namespace tests), doc comment "ten"→"twelve",
  and added group/units/range assertions for both new controls
  (`speed` / `deg`+`[0,90]` and `speed` / dimensionless+`[0,1]`). Also bumped the
  CMake "ten controls" comment and the cpp/header "nine tunables" comments to
  "twelve"/"eleven" for accuracy.
- **[suggestion] Insertion point pinned** immediately before the `cos_crab`
  division, i.e. AFTER the trajectory-speed rederivation block
  (`target_speed = segment_distance/dt.seconds();`), so regulation is not
  clobbered on timestamped trajectories. Both atomics snapshotted once just before.
- **[suggestion] Bound-check order:** the `turn_speed_min_factor` `v > 1.0`
  rejection runs BEFORE `update()` stores the atomic (out-of-range never committed).
- **[suggestion] Sim scenario closed:** waived in favor of topic-logging
  validation (recorded in `plan.md` Open Questions) — unit tests are the merge
  gate; on-water check logs `cmd_vel.linear.x` vs `crab_angle` during a turn.

### Build / test result — PASS
- Build: `colcon build --packages-up-to marine_nav_crabbing_path_follower
  --symlink-install` (build.sh needed `--packages-up-to` because the worktree
  core_ws src only carries `unh_marine_navigation`; `marine_control*` deps are
  symlinked from the main layer and had not been built). **4 packages finished,
  0 errors.** Only pre-existing warnings (unused-parameter, sign-compare in
  unrelated code); nothing from the new code.
- Test: `colcon test --packages-select marine_nav_crabbing_path_follower`.
  All gtest suites pass:
  - `test_turn_speed_factor` — **7/7 pass**
  - `test_crabbing_control` — **8/8 pass**
  - `test_gain_schedule` — 9/9 pass
  - `test_path_geometry` — 18/18 pass
  Remaining ament_lint failures (copyright, cpplint include-order, uncrustify)
  are **pre-existing package-wide conditions**: this package has never carried
  copyright headers, and the new test file matches `test_gain_schedule.cpp`'s
  include layout byte-for-byte. `ament_uncrustify` reports **no divergence** on
  the edited/new files; `ament_cpplint` reports only the same copyright +
  gtest-include-order patterns the sibling files already carry. No active
  pre-commit hook is configured for this repo worktree; commit ran clean.

### Deviations from plan.md
- Build invocation used `--packages-up-to` instead of the raw `build.sh <pkg>`
  (deps not yet built in this fresh worktree). No source-level deviation.
- Also updated the "nine"/"ten" wording in `crabbing_path_follower.{h,cpp}` and
  `CMakeLists.txt` (beyond the must-fix's test-file scope) so all count comments
  stay consistent at eleven tunables / twelve controls.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-01 02:20 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-87 at `e551513`
**Mode**: pre-push
**Depth**: Deep (reason: 242 changed code lines ≥200; safety-relevant vessel surge control + concurrency)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix findings; the change is default-off, backward-compatible, and fully unit-tested; the 3 suggestions are non-blocking and may be applied or tracked.

### Findings
- [x] (suggestion) No marine_control channel test for the two new params — `test_crabbing_control.cpp`'s `InRange`/`OutOfRangeChangeIsRejected` cover `heading_rate_gain` only; `turn_speed_min_factor`'s `v > 1.0` rejection is the only callback branch with an explicit upper-bound check and is untested at the ROS layer (pure `turnSpeedFactor` is well covered). — `test/test_crabbing_control.cpp:318,332`
- [x] (suggestion) Platform `_range` override for `turn_speed_min_factor` has no upper cap: `turn_speed_min_factor_range: [0, 5]` would advertise a FloatingPointRange up to 5.0 while the callback still rejects `> 1.0`, a panel/callback mismatch (mirrors the existing negative-`_range` defense at `crabbing_path_follower.cpp:136-137`, which caps the floor but not the ceiling). Minor — only reachable via a misconfigured platform override. — `src/crabbing_path_follower.cpp:136-137`
- [x] (suggestion) `turnSpeedFactor` consumes the *post-gain-schedule* `crab_angle` (scaled by `gainScheduleScale` at `:856-858`), so when `gain_ref_speed > 0` the regulation input is speed-scaled. Internally consistent (`cos_crab` uses the same angle) and documented in `plan.md §Context`, but not noted in the code comment at the regulation site. — `src/crabbing_path_follower.cpp:916-928`

### Notes
- **Verified false positives** (fresh-context adversarial Lens B): (a) claimed configure-time bypass of `turn_speed_min_factor`'s upper bound — the FloatingPointRange descriptor declared by `declareCrabbingControlParams` (`:162-163`) is attached *before* `read_validated` runs, so an out-of-range YAML override fails loudly at declare, exactly as the `:516-519` comment states and as every existing tunable relies on. (b) claimed multi-atomic inconsistency window between `turn_speed_max_crab_deg_` and `turn_speed_min_factor_` — no invariant binds them; `turnSpeedFactor` yields a valid factor in `[min_factor, 1.0]` for any combination, and the single-cycle snapshot is the same accepted idiom as the gain-schedule (`:857`) and lookahead (`:871-873`) pairs.
- **Confirmed correct** (both lenses): regulation factor is always `≤ 1.0` so the change can only slow the boat, never command a higher surge (safe for a vessel); NaN/Inf crab clamps to `min_factor` rather than propagating a non-finite `linear.x`; regulation is applied after the trajectory-speed rederivation and immediately before the `cos_crab` division, landing on both the commanded-speed and trajectory-speed paths.
- **Static analysis**: new hpp/test have no lines > 99 and no trailing whitespace; the two long `RCLCPP_DEBUG_STREAM` lines (`:930,:935`) match the package's existing convention (12 such lines pre-exist; `:935` is the byte-identical sibling of the prior log). No copyright headers / cpplint include-order remain pre-existing package-wide conditions, not introduced here.
- **Plan drift**: none — all 6 planned files changed, no scope creep; insertion point, `min_factor` bound-check ordering, and the 7 unit-test cases all match `plan.md` exactly.
- **Governance**: Human control & transparency (Pass — default-off, live-tunable, descriptors + finiteness guards); Consequences (Pass — tests, param declares, marine_control binding, DEBUG log all in-PR; `bindCrabbingControls` auto-binds via the `kTunables` loop, no marine_control-side change); Safety First / Simulation-First (Pass / Watch — dedicated sim scenario waived for topic-logging validation, recorded in `plan.md`); ADR-0008 ROS conventions (Pass). Copilot Adversarial off (default; `gh`/network unavailable in this worktree regardless).

## Implementation
**Status**: complete
**When**: 2026-07-01 02:36 +00:00
**By**: Claude Opus

**Branch**: feature/issue-87 at `11b2ded`
**Addressed**: `## Local Review (Pre-Push)` (approved, `feature/issue-87` at `e551513`, 2026-07-01 02:20 +00:00) — its 3 open suggestions
**Commits**: `990a0c9`, `304b3e1`, `11b2ded`

### Actions
- [x] (suggestion) marine_control channel test for `turn_speed_min_factor`'s `v > 1.0` rejection — added `CrabbingControlChannelTest.TurnSpeedMinFactorAboveOneIsRejected` (mirrors the existing `OutOfRangeChangeIsRejected` sentinel pattern: sends 2.0, then an in-range sentinel; asserts the cap held ≤ 1.0). — `test/test_crabbing_control.cpp` (`304b3e1`)
- [x] (suggestion) Platform `_range` override ceiling cap for `turn_speed_min_factor` — the declare-time range guard now rejects an override whose ceiling exceeds `default_max` **only for `turn_speed_min_factor`** (the sole tunable with a callback-enforced hard max, `v > 1.0`). A blanket ceiling cap was deliberately **not** applied: the FloatingPointRange `to_value` is itself rclcpp's settable cap, so widening it is the legitimate purpose of `_range` for tunables with no callback upper bound (gains, distances) — proven by `OutOfRangeChangeIsRejected` relying on the descriptor, not a callback branch, to reject `heading_rate_gain` > 10. Added `CrabbingControlTest.TurnSpeedMinFactorRangeCeilingClampedToDefault` ([0,5] → falls back to [0,1]). — `src/crabbing_path_follower.cpp:126-160`, `test/test_crabbing_control.cpp` (`11b2ded`)
- [x] (suggestion) Code comment at the regulation site noting `crab_angle` is the post-gain-schedule (speed-scaled) value — added a `NOTE:` paragraph before the `turnSpeedFactor` call cross-referencing `gainScheduleScale` at `:856-858` and why the scaled input is intentional/consistent with the `cos_crab` division. — `src/crabbing_path_follower.cpp:916-936` (`990a0c9`)

### Build / test result — PASS
- Build: `colcon build --packages-up-to marine_nav_crabbing_path_follower --symlink-install` — 4 packages, 0 errors (only pre-existing `-Wsign-compare` warnings in unrelated `publish_visualization`).
- Test: `colcon test --packages-select marine_nav_crabbing_path_follower` — all gtest suites pass:
  - `test_crabbing_control` — **10/10** (was 8; +2 new cases)
  - `test_turn_speed_factor` — 7/7
  - `test_gain_schedule` — 9/9
  - `test_path_geometry` — 18/18
- Remaining ctest failures (`copyright`, `cpplint`, `uncrustify`) are the **pre-existing package-wide conditions** documented in the prior `## Implementation` entry: no copyright headers anywhere in the package; uncrustify reports **"No code style divergence"** on the edited `test/test_crabbing_control.cpp` (its divergences are in pre-existing `src` code — `std::atomic < double >` spacing, DEBUG_STREAM wrapping). No added line exceeds the package's 99-char convention (verified via `git diff d485229..HEAD`). No new lint condition introduced.

### Deferred / not-actionable
- None — all three suggestions were actioned with a real code/test change.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:
`.agent/scripts/dispatch_subagent.sh --mode in-process --issue 87 --skill review-code`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-01 02:48 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-87 at `a5616d8`
**Mode**: pre-push
**Depth**: Deep (reason: safety-relevant vessel surge control + concurrency; ~311 changed code lines ≥200)
**Must-fix**: 1 | **Suggestions**: 1
**Round**: 2 | **Ship**: continue — one introduced must-fix (RCLCPP_WARN format/arg mismatch, a regression from the round-1 ceiling-clamp fix); rising 0→1 with a genuine UB defect, so another round. The fix is a single mechanical edit; one more address-findings→review-code should converge.

### Findings
- [x] (must-fix) `RCLCPP_WARN` format/argument mismatch: `%s` (`max%s`) gets a `double`, the next `%g` gets a `char*`, plus one extra unconsumed arg — UB, corrupts the malformed-`_range` diagnostic (`max(null)` + garbage), latent crash. Introduced in `11b2ded`; the `TurnSpeedMinFactorRangeCeilingClampedToDefault` test exercises this path. Fix: remove the stray `t.default_min` before the ternary. — `src/crabbing_path_follower.cpp:155`
- [x] (suggestion) Reused DEBUG line now prints `regulated_target_speed` under the label "target_speed (after potential trajectory derivation)" — now the post-regulation value; relabel or drop (the `:950` regulation log already shows both). — `src/crabbing_path_follower.cpp:955`

### Notes
- **Verified false positive** (adversarial Lens A proposed as must-fix): claimed startup upper-bound gap for `turn_speed_min_factor` (read_validated checks only `>= 0`). Refuted: `declareCrabbingControlParams` (`:483`) attaches the `[0, ≤1]` FloatingPointRange descriptor BEFORE `read_validated` (`:535`), and the `ceiling_ok` guard (`:145-148`) guarantees the descriptor ceiling can never exceed 1.0 — a YAML override >1 fails loudly at declare. Matches round-1's already-verified finding (a).
- **Confirmed safe** (both lenses): `turnSpeedFactor` returns in `[min_factor, 1.0]`, so regulation can only slow the vessel, never inflate surge; NaN/Inf crab clamps to `min_factor`, no non-finite `linear.x`; the two atomics are snapshot once per cycle with no coupling invariant (same idiom as lookahead_*/gain-schedule).
- **Static analysis**: no new lint conditions; long `RCLCPP_DEBUG_STREAM` lines (`:950,:955`, 185/195 chars) match pre-existing package convention (the `:955` sibling is already 185 chars on `origin/jazzy`). Copyright/cpplint/uncrustify remain pre-existing package-wide conditions.
- **Plan drift**: none — all planned files changed, no scope creep; the must-fix is a regression inside the round-1 ceiling-clamp fix, not a plan deviation.
- **Governance**: Human control/transparency, Consequences, Safety First, ADR-0008 all Pass; Simulation-First Watch (sim scenario waived for topic-logging, unchanged from round 1).

### Next step
Verdict is **changes-requested** → host (`/run-issue`) dispatches **address-findings** to work the open must-fix + suggestion from this entry, then re-dispatches **review-code**. The diff is not pushed until a pre-push review comes back approved.
