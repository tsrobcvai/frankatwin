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
