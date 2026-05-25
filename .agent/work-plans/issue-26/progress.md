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
- [ ] Add `std::isfinite` check on the BT `speed` input in `SetControllerSpeed::tick()` before the `<= 0.0` guard; skip with throttled WARN if non-finite (Copilot #1).
- [ ] In `CrabbingPathFollower::on_set_parameters_callback`, reject non-finite or non-positive `default_speed` by returning `SetParametersResult{successful=false, reason="..."}` (Copilot #9, paired defense-in-depth with the BT-side check).
- [ ] Treat empty `target_node` input in `SetControllerSpeed::tick()` as an error — throw `BT::RuntimeError` to match the "missing" case, or return SUCCESS with explicit WARN (Copilot #2).
- [ ] Add explicit `#include "rcl_interfaces/msg/set_parameters_result.hpp"` to `set_controller_speed.cpp` — currently relies on transitive include from rclcpp (Copilot #4).
- [ ] Remove custom `main()` from `test_set_controller_speed_resolve.cpp`; rely on `gtest_main` linked by `ament_add_gtest()` (Copilot #5 + #8).
- [ ] Add `<Action ID="SetControllerSpeed">` entry to the inline `<TreeNodesModel>` block in `run_tasks.xml` (after line 402) to match the sibling plugin entries — fixes both Copilot #6 and #7 (same finding on two insertion points).
- [ ] (Optional) Reply to Copilot #3 (`last_pushed_speed_` update timing) explaining the false-positive rationale: resetting on failure would flood the unthrottled rejection WARN on persistent validator rejection AND introduce a thread race against the BT thread's non-atomic read.
