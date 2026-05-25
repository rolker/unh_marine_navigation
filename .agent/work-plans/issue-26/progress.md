---
issue: 26
---

# Issue #26 — Field import: per-task speed plumbing (2026-05-22)

## External Review
**Status**: complete
**When**: 2026-05-25 11:40
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #27 — 1 review, 9 inline comments, 6 valid (after collapsing duplicates), 1 false positive
**CI**: all-pass (`copilot-pull-request-reviewer` → success)

### Actions
- [x] Add `std::isfinite` check on the BT `speed` input in `SetControllerSpeed::tick()` before the `<= 0.0` guard; skip with throttled WARN if non-finite (Copilot #1).
- [x] In `CrabbingPathFollower::on_set_parameters_callback`, reject non-finite or non-positive `default_speed` by returning `SetParametersResult{successful=false, reason="..."}` (Copilot #9, paired defense-in-depth with the BT-side check).
- [x] Treat empty `target_node` input in `SetControllerSpeed::tick()` as an error — throw `BT::RuntimeError` to match the "missing" case (Copilot #2).
- [x] Add explicit `#include "rcl_interfaces/msg/set_parameters_result.hpp"` to `set_controller_speed.cpp` (Copilot #4).
- [x] Remove custom `main()` from `test_set_controller_speed_resolve.cpp`; rely on `gtest_main` linked by `ament_add_gtest()` (Copilot #5 + #8).
- [x] Add `<Action ID="SetControllerSpeed">` entry to the inline `<TreeNodesModel>` block in `run_tasks.xml` to match the sibling plugin entries — fixes both Copilot #6 and #7.
- [ ] (Optional) Reply to Copilot #3 (`last_pushed_speed_` update timing) explaining the false-positive rationale: resetting on failure would flood the unthrottled rejection WARN on persistent validator rejection AND introduce a thread race against the BT thread's non-atomic read.

All valid findings addressed in commit (pending). 5/5 gtest cases still pass; build clean; no new lint findings introduced.
