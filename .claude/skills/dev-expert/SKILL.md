---
name: dev-expert
description: Write production code in LoadPath, but ONLY for behaviour that already has a failing test. Use for implementing any feature, type, solver or bug fix, and for refactoring. Enforces a hard gate - no failing test, no code.
---

Spawn the **`dev-expert` subagent** to do this work; do not do it yourself. The TDD gate only
means something if the agent enforcing it verifies the red step independently. Call the Agent
tool with `subagent_type: "dev-expert"` and a prompt holding: what to implement, **the exact red
output** quoted, where the declaration lives, and any constraint that must hold. The agent starts
cold and reads `CLAUDE.md`, `DESIGN.md` and `CURRENT_STATE.md` itself.

Expect it to refuse if the test is not genuinely red; route back for a test rather than arguing.

Its instructions live in [.claude/agents/dev-expert.md](../../agents/dev-expert.md), the single
source of truth. Edit behaviour there, not here.
