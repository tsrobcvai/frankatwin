# TODO

Work the authors owe this repository — not instructions for people using it.
Anything a user needs to know belongs in `docs/`; if a note here turns into
guidance, move it there and delete the entry.

## Documentation

- [ ] **Visualize the trajectories.** The three reference shapes in Example 2 of
      [Usage](docs/usage.md) (`sine`, `multiband`, `chirp`) are described in
      prose only. Add a plot of each so the shape, amplitude and duration can be
      seen before running it on the robot. `scripts/plot_ee_tracking.py` already
      plots actual-vs-target from a log, and `cart_impedance.py --dry-run`
      builds the reference without a robot, so the figures can be generated
      offline.
- [ ] **Check the shared-config paragraph in Installation.** Pulled out of a
      `[TODO, checklist]` box in [Installation](docs/installation.md), so it was
      never signed off. The text read: NUC and PC read the same
      `config/robot.yaml` from their checkout (`pip install -e .` is the
      intended install mode): `robot.ip` for the robot, `network.nuc_host` for
      the NUC, plus gains, safety clamps and payload. Override with `--config`
      on any command or `FRANKATWIN_CONFIG=/path/to/local.yaml`; keys are
      documented inline in the file and in
      [Configuration](docs/configuration.md). Verify it against
      `python/frankatwin/config.py` and put it back as ordinary prose, or drop
      it if [Configuration](docs/configuration.md) already covers it.
