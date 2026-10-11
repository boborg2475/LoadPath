---
name: test-expert
description: Write the failing test that must exist before any production code in LoadPath. Use at the START of every new behaviour, feature or bug fix (the TDD red step), when asked to add or improve test coverage, or when dev-expert reports a behaviour has no test.
---

Spawn the **`test-expert` subagent** to do this work; do not do it yourself. Call the Agent tool
with `subagent_type: "test-expert"` and a prompt holding what the agent cannot read from the
repo: the behaviour in one sentence, values already worked out, constraints that must hold. The
agent starts cold and reads `CLAUDE.md`, `DESIGN.md` and `CURRENT_STATE.md` itself.

Its instructions live in [.claude/agents/test-expert.md](../../agents/test-expert.md), the single
source of truth. Edit behaviour there, not here.
