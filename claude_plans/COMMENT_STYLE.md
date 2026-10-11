# Comment style

Owner's rule, 2026-10-10. Supersedes the old project's rubric entirely.

1. **`.cpp` files carry no comments.** Functions, bodies, tests: none. If a body needs a comment
   to be understood, split it or rename it.
2. **Headers may carry one or two sentences per declaration**, as `//` lines directly above it,
   only when they add what the name cannot: a unit trap, a fail-closed reason, an invariant, a
   cross-reference to DESIGN.md or TRAPS.md.
3. **No prose blocks anywhere.** No paragraphs, no history, no TDD narrative, no restating the
   signature.
4. **Provenance goes in [ANCHORS.md](ANCHORS.md)**, keyed by test name: where a pinned number
   came from, the formula, the date measured.

Reviewers treat a comment in a `.cpp`, or a header comment past two sentences, as a finding.
