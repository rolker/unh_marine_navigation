---
issue: 9
---

# Issue #9 — CI workflow for unh_marine_navigation

## Issue Review
**Status**: complete
**When**: 2026-05-26 21:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #9
**Comment**: https://github.com/rolker/unh_marine_navigation/issues/9#issuecomment-4550504392
**Scope verdict**: well-scoped

### Actions
- [ ] Adopt the workspace CI template `.agent/templates/ci_workflow.yml` (cascade principle) rather than hand-rolling; fill placeholders: branches `jazzy`, repo name `unh_marine_navigation`, image `ros:jazzy-ros-core`, package list = the 7 `marine_nav_*` packages.
- [ ] Remove the template's optional `git clone` block — all deps resolve via rosdep for Jazzy (`nav2_*`, `behaviortree_cpp`, `control_toolbox`, `tf2`, msgs); internal `marine_nav_*` deps handled by `--packages-up-to`.
- [ ] Land early — don't gate on #3–#8. CI runs a meaningful build + `ament_lint_auto` pass immediately and gates subsequent test PRs as they merge.
- [ ] Surface separately whether to promote the workflow to a *required* status check — branch-protection change is an AGENTS.md Ask-First / human-approval item; don't flip silently.
- [ ] Keep the hardcoded default branch (`jazzy`) + package list in sync as packages are added/renamed.
