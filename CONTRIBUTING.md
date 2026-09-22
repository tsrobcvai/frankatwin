# Contributing

Thanks for taking the time. FrankaTwin is small on purpose: a 1 kHz C++
controller, a thin Python API, and a sysid pipeline whose sim controller is a
line-for-line mirror of the real one. Changes that keep that mirror intact are
the ones we can merge.

## Ground rules

- **The real and sim controllers must stay identical.** If you change the
  control law in `src/osc_shm.cpp`, change
  `isaaclab_sysid/.../franka_sysid/control.py` in the same PR and say so.
- **The shm layout is an ABI.** Any change to `src/shm_layout.h` must bump
  `FRANKATWIN_SHM_VERSION`, update `python/frankatwin/shm_layout.py`, and pass
  `tests/test_shm_layout.py` (which compiles the C++ struct and diffs offsets).
- **Safety defaults are not tunables.** `TAU_LIMIT`, the torque slew limiter and
  the collision thresholds exist because of real incidents documented in
  [docs/troubleshooting.md](docs/troubleshooting.md). Loosening them needs a
  written rationale.
- No hard-coded machine paths, IPs other than the Franka defaults
  (`172.16.0.x`), or lab-specific names.

## Dev setup

```bash
pip install -e ".[test,analysis]"
python -m pytest tests -q          # needs g++, no libfranka
```

The C++ side needs libfranka and a robot; see
[docs/installation.md](docs/installation.md).

## Docs

The site under `docs/` is Sphinx (sphinx-book-theme, Markdown via MyST). Preview it
locally with live reload — no GitHub involvement:

```bash
pip install -e ".[docs]"
sphinx-autobuild docs docs/_build/html      # http://127.0.0.1:8000
```

`docs/index.md` is the landing page and holds the two sidebar sections
(Guide, Reference). The guide pages are the same `docs/*.md` you see on GitHub;
API pages are generated from docstrings (`docs/api.md`).

## Pull requests

- One logical change per PR, conventional-commit style subject
  (`feat(daemon): …`, `fix(osc_shm): …`, `docs: …`).
- If you touched anything that runs on the robot, include the command you ran
  and the daemon banner / tracking summary in the PR description.
- Add a line to [CHANGELOG.md](CHANGELOG.md) under *Unreleased*.
