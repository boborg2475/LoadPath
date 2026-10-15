# Traps

Footguns carried from `DestructionGame`'s `claude_plans/TRAPS.md`, kept to the ones that apply
before a line of this project exists. Each cost real time there. Add a new one the day it bites.

## Arithmetic and fail-closed

- **NaN is laundered by min and max.** `(a < b) ? a : b` with a NaN picks one operand and moves
  on, so a NaN becomes a plausible capacity. Spell guards `!(x > 0.0)`, never `x <= 0.0`, so the
  NaN lands inside the guard. The break comparison is `!(ratio <= 1.0)`. The general rule: the
  guard puts NaN on the fail-closed side, whichever side that is; when true is the fail-open
  answer the polarity flips.
- **Fail-closed for a mass is NaN, never zero.** A zero mass is a legitimate massless piece.
- **A value safe as an answer is unsafe as a command.** "Falling" as the default support state
  promises least in a readout and triggers irreversible release one layer up. Anything
  irreversible first asks whether there is an answer at all.
- **The worst-axis number hides which axis won.** A utilisation reported as one number was
  mis-attributed by eye twice, each time plausibly. The verdict names the governing axis, and an
  axis claim in prose with no formula beside it is unverified.
- **A lower strength makes a break more likely, not less.** A demand-over-capacity ratio moves
  inversely with capacity. Work the direction; do not scale the number.

## Geometry

- **The interface normal is the axis of separation, oriented by which body is second, never the
  centroid direction.** A running-bond bed joint's centroid direction has |z| = 0.55, under
  cos 45°, so a centroid normal classified every bed joint in a wall as a head joint with
  nothing crashing. Emit the pair, the normal, the area and the extents as one value from one
  function.
- **An overlap is an interval intersection, `min(highs) - max(lows)`, never `reach - distance`.**
  The two agree for equal boxes and diverge silently when one span contains the other, with the
  area and the rectangle wrong together so a consistency check passes.
- **In-plane axes derived from the normal's smallest component put U out of the plane** for
  every normal except the one you tested with. Pair corners by a world axis when the pairing
  matters.

## Model

- **Giving is irreversible and a given joint carries exactly nothing.** A broken joint leaves
  every relation at once; half-right is a wall that collapses where it should have leaned.
- **Pass numbers belong to the structure, not the call.** A later cascade continues from the
  highest stamp.
- **Handles are stable across removal.** Removal tombstones; nothing renumbers.
- **Breaking every over-capacity joint at once diverges on large structures.** The old
  warehouse ran damage to the far wall. Crack the top band of ratio per pass; sever by the cut.
- **A cluster released as one body must be a certified mechanism.** Welding a reachability set
  into one body hung it in the wall.

## Strength data

- **Mean values, not characteristic design values**, for realism; code figures are 5% fractiles
  meant to sit under safety factors. The mean/characteristic ratio is not uniform (tension ×7,
  cohesion ×4.5, compression ×1), so the governing axis can flip at a re-anchor; re-measure, never
  re-scale. Every profile row cites its source.

## Testing

- **A green-on-arrival test is indistinguishable from one that asserts nothing** until
  production is mutated. Prove it bites; record the mutation.
- **An invariant over fixtures that share a hidden property is not an invariant.**
- **Hand-written cases cover the shapes someone thought of.** Both real solver defects in the old
  project lived in topologies nobody wrote by hand; a seeded fuzz against an independently
  derived oracle found them.
- **An opt-in test tier rots.** Name the milestone that runs it.
- **Never revert a test mutation with `git checkout --` on a file carrying uncommitted work.**
  Re-apply the line.
- **`grep -c $'\r'` is useless in Git Bash** for line-ending checks; use `tr -cd '\r' | wc -c`.
- **`REQUIRE` does not stop a test here.** Under `DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS`
  a failed `REQUIRE` records the failure and carries on, so indexing after it can crash the
  process (a SIGSEGV on `Calls[0]` after a failed size check, 2026-10-15). Guard the access with
  an `if` after the check.
- **A `Structure` copy takes a fresh lineage**, so a warm re-judge on a copy reuses nothing. Judge
  cold and warm on the same object when a test needs reuse.
- **Two builds into one folder at once collide** (C1041 on the shared `.pdb`). Agents running in
  parallel use different build folders: the main tree and the scratch worktree.
