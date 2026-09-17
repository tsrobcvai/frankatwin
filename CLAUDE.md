# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Verify against the source before documenting

Every factual claim written into `docs/`, `README.md`, a docstring or a header
comment must be checked against the code it describes — read the implementation,
not the surrounding prose. Existing comments and docs are evidence of intent,
not of behaviour: they go stale, and copying a stale claim forward spreads it.

Read, in particular:

- the algorithm itself, not its summary, when documenting *how* a motion or
  controller behaves (`src/*.cpp`, including the vendored `examples_common.cpp`);
- the argument parser, when documenting flags, their ranges and defaults
  (`examples/*.py`, `parse_args` in `src/move_to.cpp`);
- `config/robot.yaml`, when quoting a default that comes from configuration.

Where a claim cannot be verified without hardware, say what was checked and what
was not, rather than asserting the untested part.

A concrete miss to learn from: `move_to --target-joints` was documented for a
long time as following a "min-jerk profile". It does not — it runs libfranka's
`MotionGenerator`, a per-joint cubic ramp up, constant-velocity cruise and cubic
ramp down (a smoothed trapezoid), synchronized across the seven joints. Only the
`--target-ee` path is genuinely 5th-order min-jerk (`minjerk_s` in
`src/move_to.cpp`). The wrong claim sat in the header comment and was copied
into the docs from there.

## Language

The repository — code, comments, docs, commit messages and log output — is
written in English, even when the working conversation is in another language.
