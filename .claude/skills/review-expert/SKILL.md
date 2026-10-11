---
name: review-expert
description: Review completed LoadPath work before it is called done. Use after dev-expert has a green test, before committing, or when asked to review a change, a test or a design decision. Expected to reject work, not rubber-stamp it.
---

Spawn the **`review-expert` subagent** to do this work; do not do it yourself. Call the Agent
tool with `subagent_type: "review-expert"` and a prompt holding: what was changed and why, the
test that drove it, and the claimed test results. The agent re-runs everything itself and has no
write access by design. Act on blocking findings; log deferred ones in `CURRENT_STATE.md`.

Its instructions live in [.claude/agents/review-expert.md](../../agents/review-expert.md), the
single source of truth. Edit behaviour there, not here.
