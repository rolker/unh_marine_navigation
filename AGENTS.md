# AGENTS.md — unh_marine_navigation

Instructions for AI agents working in this repository — including **GitHub
Copilot code review**, which reads this file when reviewing PRs. There is no
`.agents/README.md` deep guide yet (the workspace convention for project
agent guides — distinct from the `.agent/work-plans/` directory); start from
the top-level `README.md` and the package READMEs.

## Workspace Rules

This repo is developed inside a
[ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace).
The workspace root `AGENTS.md` carries the full shared rules (worktree
isolation, issue-first policy, commit conventions, AI signatures). This file
**references** those rules and adds repo-specific context only — it must
never restate or fork them.

## Quality Standard

This is software for autonomous robot boats operating on open water.
Robustness is not optional.

- Fix bugs completely: add the test, handle the edge case, check the
  lifecycle transition.
- Concerns about error handling, silent failures, stale data, or missing
  validation are not nits — flag them unless the failure mode genuinely
  cannot occur. "Config is under our control" and "pathological input" are
  not blanket dismissals; field configs change under pressure.
- A change includes its consequences: tests, documentation, and dependent
  references update in the same PR.

## Reviewing PRs

- If the PR carries a work plan (`.agent/work-plans/issue-<N>/plan.md` or a
  plan in the PR body), the plan is kept **in sync with the implementation
  as it evolves** — an implementation that matches the current plan text is
  not "plan drift", even if the plan changed after the PR opened.
- Verify claims against source: parameters, topics, services, and message
  types in docs must match the code.

## Review Context — unh_marine_navigation

- **Safety-critical**: this stack steers real uncrewed boats — the Nav2 /
  BT.CPP task navigator and controllers publish `cmd_vel` into the live helm
  chain. Controller and behavior-tree changes need tests and, where they
  alter motion, simulation verification.
- **Marine dynamics are deliberate, not defaults**: control/update rates
  around 10 Hz and slow vehicle response are correct for these vessels —
  don't flag them against small-robot (20+ Hz) expectations.
- **Per-pose path timestamps are load-bearing**: `CrabbingPathFollower`
  derives segment speed from pose stamps (distance/Δstamp overrides
  `default_speed`); stamp changes change boat speed.
- **The behavior tree latches the followed path** (`run_tasks.xml` family);
  task re-entry and mission re-send semantics are subtle — treat changes to
  task gating with extra suspicion.
