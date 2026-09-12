<p align="center">
  <img src="docs/images/logo.png" width="440" alt="FrankaTwin — sim ↔ real">
</p>

<p align="center">
  <a href="https://tsrobcvai.github.io/frankatwin/"><b>Documentation</b></a>
</p>

<p align="center">
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/license-Apache--2.0-blue.svg"></a>
  <img alt="libfranka" src="https://img.shields.io/badge/libfranka-0.13%20%E2%80%93%200.15-informational">
  <img alt="IsaacLab" src="https://img.shields.io/badge/IsaacLab-%E2%89%A5%202.3-76b900">
  <img alt="Python" src="https://img.shields.io/badge/python-%E2%89%A5%203.9-3776ab">
</p>

---

**FrankaTwin aligns the Franka Research 3 / Panda between simulation and reality.**

- **One task impedance controller, real and simulated.** The same 1 kHz
  task-space impedance law on the robot (C++, libfranka) and in IsaacLab — same
  gains, damping and torque limits.
- **A reliable system-identification pipeline for Isaac Sim.** Fits the arm's
  joint dynamics from real excitation runs; the identified sim tracks the real
  arm to a joint-position MSE of 4.8 × 10⁻⁴ rad² (1–3 % of joint range) on
  held-out motions.

Built for training and evaluating policies that transfer. The control scheme
matches the task-space impedance used by sim-to-real work such as
[IndustReal](https://arxiv.org/abs/2305.17110) and
[OmniReset](https://weirdlabuw.github.io/omnireset/).

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
- **Harry He** — McGill University

## License

Apache-2.0. Vendored code from libfranka (Apache-2.0) and Isaac Lab (BSD-3-Clause)
is listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
