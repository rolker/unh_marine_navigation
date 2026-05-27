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
