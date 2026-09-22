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

## What you can check without a robot

Everything but the hardware paths:

```bash
python -m pytest tests -q                                      # 76 tests, no robot, no Isaac Sim
python -m py_compile examples/*.py scripts/*.py isaaclab_sysid/scripts/tools/*.py
python examples/cart_impedance.py --mode chirp --dry-run       # builds the reference, sends nothing
sphinx-build -b html -n --keep-going docs docs/_build/html     # pip install -r docs/requirements.txt
```

The tests put `python/` on `sys.path` themselves, so they run in a bare
checkout; the examples and scripts do not, so export `PYTHONPATH=python` or
`pip install -e ".[test]"` before running those. Install the `test` extra to
get all 76 — without `pandas` and `matplotlib` the comparison tests skip.

Nothing here covers libfranka or Isaac Sim. A change under `src/` or
`isaaclab_sysid/` is verified by reading it and by running it on the hardware,
so keep those small and say in the commit message what was run and what was
not.

## `isaaclab_sysid/` is installed by copy

`install_into_isaaclab.sh` copies the two tasks, `franka_mimic.usd` and the
four `scripts/tools/*.py` into an IsaacLab checkout. The copies drift silently.
This repository is the source of truth: edit here, then re-run the installer —
never edit the installed copy and hope to bring it back.

## Where things already are

`docs/architecture.md` holds the control law, the shm layout, the daemon's
behaviour and wire protocol, the safety chain, the IsaacLab task ids and the
repository map; `docs/data_format.md` holds every file schema. Both are kept
current — read them instead of restating them here, and when a fact moves, it
moves rather than being written down twice.
