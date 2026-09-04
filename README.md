<p align="center">
  <img src="docs/images/logo.png" width="440" alt="FrankaTwin — sim ↔ real">
</p>

<p align="center">
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/license-Apache--2.0-blue.svg"></a>
  <img alt="libfranka" src="https://img.shields.io/badge/libfranka-0.13%20%E2%80%93%200.15-informational">
  <img alt="IsaacLab" src="https://img.shields.io/badge/IsaacLab-%E2%89%A5%202.3-76b900">
  <img alt="Python" src="https://img.shields.io/badge/python-%E2%89%A5%203.9-3776ab">
</p>

---

FrankaTwin is a 1 kHz task impedance controller for the Franka Research 3 / Panda
(C++ on libfranka, Python client over ZMQ) with a system-identification pipeline
that fits the arm's joint dynamics in IsaacLab. Real and simulated controllers
share one control law — the task-space impedance used by sim-to-real work such as
[IndustReal](https://arxiv.org/abs/2305.17110) and
[OmniReset](https://weirdlabuw.github.io/omnireset/) — and the identified
dynamics track the real arm to a joint-position MSE of 4.8 × 10⁻⁴ rad² on a
held-out 6-DOF chirp.

## Documentation

Everything lives in [`docs/`](docs/):
[Quick start](docs/quickstart.md) ·
[Installation](docs/installation.md) ·
[Usage](docs/usage.md) ·
[Architecture](docs/architecture.md) ·
[System identification](docs/sysid.md) ·
[Data format](docs/data_format.md) ·
[Troubleshooting](docs/troubleshooting.md) ·
[Interfaces](docs/interfaces.md)

Browse it as a site with `pip install -e ".[docs]" && sphinx-autobuild docs docs/_build/html`
(includes the API reference generated from docstrings).

## Citing

```bibtex
@software{frankatwin2026,
  author  = {Sun, Tao and Yin, Patrick},
  title   = {FrankaTwin: a sim-to-real aligned 1 kHz task impedance controller for the Franka Research 3},
  year    = {2026},
  version = {0.2.0},
  url     = {https://github.com/tsrobcvai/frankatwin}
}
```

## Authors

- [**Tao Sun**](https://taosun99.github.io/) — McGill University
- [**Patrick Yin**](https://patrickyin.me/) — University of Washington

## License

Apache-2.0. Vendored code from libfranka (Apache-2.0) and Isaac Lab (BSD-3-Clause)
is listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
