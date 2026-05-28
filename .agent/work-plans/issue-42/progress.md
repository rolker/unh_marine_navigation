---
issue: 42
---

# Issue #42 — `/<ns>/hover_visualization` has two DDS type announcements — `ros2 bag record` refuses to capture it

## Implementation
**Status**: complete (code) — sim verification deferred (matches PR #40's pragmatic deferral choice)
**When**: 2026-05-28 11:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Branch**: `feature/issue-42`
**Diff scope**: single-line guard + comment block — `marine_nav_behaviors/src/hover.cpp` (+10 / −1).

### Root cause (from Explore-subagent investigation 2026-05-28)

`hover.cpp:44-48` recreated the `LifecyclePublisher<MarkerArray>` inside `Hover::onRun()` on every behavior invocation:

```cpp
if(generate_visualization_)
{
  visualization_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    behavior_name_+"_visualization", 1);
  visualization_publisher_->on_activate();
}
```

When the hover behavior is invoked multiple times (operator hover-cancel-hover cycle; BT dispatch re-entering `HoverTask`), the prior publisher's `shared_ptr` is overwritten by the new one. The old publisher is destroyed while the new one is constructed. DDS topic-type discovery has a small TTL, so during the transient window both type registrations are live for the same topic name. `ros2 bag record` does strict type-uniqueness validation at subscribe time and rejects the topic with `more than one type associated`.

### Fix

Added `&& !visualization_publisher_` guard so the publisher is created exactly once (lazy first-call) and reused across all subsequent invocations. No recreation, no transient overlap, no recorder rejection.

Comment block added explaining the race + the reason for the guard, citing #42.

### What was NOT changed (and why)

- **Moving to `onConfigure`**: considered (the original Explore-subagent recommendation). Decided against because lazy-create-on-first-use preserves the runtime-toggleable `generate_visualization` parameter semantics (operator can `ros2 param set ... generate_visualization=true` after configure). Guard achieves the no-race property without removing flexibility.
- **Unit test for "publisher created only once"**: considered. Adding a test would require ~50-100 lines of lifecycle-node fixture (rclcpp init, lifecycle transitions, parameter setting, mock node, pointer introspection) for a 1-line defensive guard. The probability of a regression that removes the guard is low; the cost-benefit doesn't justify the infrastructure. Sim verification (below) is the actual acceptance test.
- **Uncrustify drift on the rest of `hover.cpp`**: pre-existing package-wide style drift (uncrustify 0.78.1 wants `+` operator spaces, K&R braces; file is in Allman style throughout). My added line matches the file's existing style. Out of scope for this PR — mirrors the precedent set by PR #41 / PR #37 commit messages.

### Sim verification — DEFERRED

The verification scenario: launch project11 sim, trigger `HoverTask` multiple times (operator hover-cancel-hover, ≥3 cycles), `ros2 bag record /<ns>/hover_visualization` in parallel, confirm:
- No "more than one type associated" ERROR in the recorder log.
- The bag captures `MarkerArray` messages on the topic (`ros2 bag info ...` shows non-zero message count).

Deferring per the same pragmatic choice as PR #40 (June 4 freeze pressure). The mechanism is well-understood from source-read + the DDS discovery model; the guard is a one-line defensive change. Sim verification can be scheduled before / during the next field deployment, alongside #40's continuity repro.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-28 11:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: `feature/issue-42` at `<sha>` (TBD on commit)
**Mode**: pre-push (base `origin/jazzy`)
**Depth**: Light-equivalent — 1-line defensive fix; mechanism evidenced by source-read + DDS discovery model.
**Must-fix**: 0 | **Suggestions**: 1 (deferred sim verification, see Implementation)

### Findings
- [ ] (suggestion, deferred) Sim-verification scenario above. To be run before / during the next field deployment.
