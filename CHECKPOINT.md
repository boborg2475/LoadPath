# certificate_checkpoint

Branched from master at 05df4ea on 2026-11-08, before the solver architecture changes.

This branch keeps the two-solver design as it stood: the RBSM elastic authority and the
limit-analysis certificate combined by `CombinedAuthority`, with the certificate as the
referee for Mechanism, Undecided and only-crushed components and the sole source of the
release list. The owner is changing the solvers on master (a common solver interface, a chain
authority with its policy as data, an elastic-only runtime), so this is the reference point for
comparing verdicts and timings against the old arrangement.

Do not develop on this branch.
