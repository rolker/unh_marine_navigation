---
issue: 38
---

# Issue #38 — Cropped local-costmap republisher for operator display

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 11:55 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: feature/issue-38 at `8c912e9`
**Mode**: pre-push
**Depth**: Standard (reason: new node with non-trivial crop/origin math)
**Must-fix**: 0 remaining (5 found and fixed in-branch) | **Suggestions**: 0 open

### Findings
- [x] (must-fix) NaN/non-positive window size — `std::clamp` UB, collapsed to 0x0 grid — `src/costmap_window.cpp:20`
- [x] (must-fix) non-finite resolution slipped past `<= 0` guard into `std::clamp` — `src/costmap_window.cpp:20`
- [x] (must-fix) `info.map_load_time` dropped; now copies whole `info` then overrides — `src/costmap_window.cpp:45`
- [x] (must-fix) row copy assumed `data.size()==width*height`; added bounds guard — `src/costmap_window.cpp:28`
- [x] (suggestion) non-finite quaternion → NaN origin; folded into identity guard — `src/costmap_window.cpp:63`
- [x] (suggestion) subscription reliability now best-available (was default reliable) — `src/costmap_window_node.cpp:28`
- [ ] (rejected) pub best_effort for "lossy path" — false positive: this DDS hop is node→bridge, both local on gabby; lossiness is downstream of the bridge

## Integrated Review
**Status**: complete
**When**: 2026-05-27 12:31 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #39 at `ad1106c`
**Sources**: 4 (Copilot R2 @ `ad1106c`, Copilot R1 @ `94898b0` [stale], Local Review (Pre-Push) @ `8c912e9`, CI rollup)
**Cross-source confirmations**: 0 (D & E raised in both Copilot rounds — same source, persistence signal)
**CI**: all-pass (copilot reviewer check; no build/test CI on this repo yet — issue #9)

### Findings
- [ ] (valid, Copilot R1+R2) package.xml missing `tf2` / `tf2_geometry_msgs` `<depend>` (linked + included, undeclared) — `marine_nav_utilities/package.xml`
- [ ] (valid, Copilot R2) `window_size_` data race under a MultiThreadedExecutor + non-default callback group; make `std::atomic<double>` — `marine_nav_utilities/include/marine_nav_utilities/costmap_window_node.h`
- [ ] (valid, Copilot R1+R2) docstring omits non-finite-resolution/window and data.size-mismatch no-op cases — `marine_nav_utilities/include/marine_nav_utilities/costmap_window.h`
- [ ] (valid-low, Copilot R1) invalid origin quaternion: offset uses identity but output orientation left as invalid input; write identity on guard — `marine_nav_utilities/src/costmap_window.cpp`
- [ ] (test-gap, Copilot R1) no test for invalid-quaternion→identity path — `marine_nav_utilities/test/test_costmap_window.cpp`
- [ ] (decision, from Copilot R2 FP) bare-integer `param set` is rejected by static typing; decide whether to support ints via dynamic typing + coercion — `marine_nav_utilities/include/marine_nav_utilities/costmap_window_node.h`

### False positives
- (Copilot R2) `OnSetParametersCallbackHandle::SharedPtr` "likely to fail to compile" — false: `rclcpp::Node` aliases it (node.hpp:1014), resolves via base-class lookup; executable is built.
- (Copilot R2) `as_double()` "will throw and can crash" on integer set — false: static parameter typing rejects a non-double with "Wrong parameter type" before the on-set callback runs; verified at runtime, node stayed alive.
