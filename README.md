<p align="center">
  <img src="docs/images/logo.png" width="440" alt="FrankaTwin — sim ↔ real">
</p>

<h2 align="center"><a href="https://tsrobcvai.github.io/frankatwin/">Documentation</a></h2>

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
  gains and damping.
- **A reliable system-identification pipeline for Isaac Sim.** Fits the arm's
  joint dynamics from real excitation runs; on a held-out chirp at gains the
  fit never saw, the identified sim tracks the real arm to 3.6 mm end-effector
  position RMSE and 28 mrad joint-position RMSE — 5.4× and 16.5× better than
  the PhysX defaults.

https://github.com/user-attachments/assets/226f0dac-2274-48f2-9b4c-02b531a0747f

*The IsaacLab twin (left) beside the real arm (right) on that held-out chirp,
with and without the identified parameters; the overlay tracks the sim-to-real
error live.*

Built for training and evaluating policies that transfer. The control scheme
matches the task-space impedance used by sim-to-real work such as
[IndustReal](https://arxiv.org/abs/2305.17110) and
[OmniReset](https://weirdlabuw.github.io/omnireset/).

## Citation

If you use FrankaTwin in your research, please cite the software:

<!-- Once a Zenodo DOI exists, add `doi = {10.5281/zenodo.<id>}` and
     `publisher = {Zenodo}` to this entry, and `doi:` to CITATION.cff. -->
```bibtex
@software{sun_frankatwin_2026,
  author  = {Sun, Tao and Yin, Patrick and He, Harry},
  title   = {FrankaTwin: Sim-to-Real Aligned Control and System Identification for Franka Robots},
  year    = {2026},
  version = {0.3.0},
  url     = {https://github.com/tsrobcvai/frankatwin}
}
```

FrankaTwin was developed in support of our visual sim-to-real manipulation
research. If you use the controller, methodology, or experimental setup in this
context, please also cite:

```bibtex
@misc{sun2026visualsimtoreallearningrobotic,
  title={Visual Sim-to-Real Learning for Robotic Insertion under Geometric Variations: Application to Rebar Installation},
  author={Tao Sun and Beining Han and Patrick Yin and Rui Xu and Harry He and Abhishek Gupta and Szymon Rusinkiewicz and Yi Shao},
  year={2026},
  eprint={2609.20477},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2609.20477}
}
```

## Authors

- [**Tao Sun**](https://taosun99.github.io/) — McGill University
- [**Patrick Yin**](https://patrickyin.me/) — University of Washington
- **Harry He** — McGill University

## License

Apache-2.0. Vendored code from libfranka (Apache-2.0) and Isaac Lab (BSD-3-Clause)
is listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
