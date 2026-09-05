# Installation

Three machines can be involved. Every command below is tagged with where it runs:

| tag | machine | runs | software (tested) |
|---|---|---|---|
| — | **Robot** | — | Franka Research 3 (system ≥ 5.7) or Panda, Franka Hand attached, FCI enabled in Desk |
| <kbd>NUC</kbd> | real-time PC wired to the robot (FCI) | `python -m frankatwin.daemon` → `osc_shm` / `move_to` | Ubuntu 20.04 / 22.04 with `PREEMPT_RT` kernel · libfranka 0.13–0.15; ≥ 0.14 needs Pinocchio, handled by CMake · Eigen3, CMake ≥ 3.10 · Python ≥ 3.9 |
| <kbd>PC</kbd> | your workstation | `examples/*.py`, analysis scripts | Python ≥ 3.9 (numpy, pyyaml, pyzmq; pandas + matplotlib for the analysis scripts) |
| <kbd>SIM</kbd> | any GPU box with IsaacLab (can be the PC) | sysid fit, sim replay | IsaacLab 2.3.0 (≥ 2.3 for the dynamic/viscous joint-friction API) · `cmaes` |

## At a glance

The whole installation in two blocks; the sections below explain each line.

Full prerequisites (RT kernel, FCI, libfranka ≥ 0.14 + Pinocchio, conda caveats):
[docs/installation.md](installation.md).

<kbd>NUC</kbd> build the 1 kHz controller, install the Python side, start the daemon

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
cmake -S . -B build && cmake --build build -j      # libfranka + Eigen3 (+ Pinocchio for libfranka >= 0.14)
pip install -e .
python -m frankatwin.doctor        # RT kernel, rtprio, binaries, libfranka/pinocchio, FCI link, other FCI clients
python -m frankatwin.daemon        # binds 5555 (commands) / 5556 (state), launches osc_shm
```

<kbd>PC</kbd> Python only

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
pip install -e ".[analysis]"          # analysis: pandas + matplotlib for the compare/plot scripts
vim config/robot.yaml                 # network.nuc_host = the NUC's address as seen from here
python -m frankatwin.doctor                     # daemon reachable? state stream flowing?
```

<kbd>SIM</kbd> only if you will run the sysid loop — see [System identification](sysid.md).

`config/robot.yaml` is shared by all sides (network, robot IP, gains, safety
clamps, collision thresholds, payload). Override with `--config` or
`$FRANKATWIN_CONFIG`.

## 1. NUC prerequisites

You need what every libfranka user needs: a `PREEMPT_RT` kernel, the FCI feature
enabled in Desk, a user in the `realtime` group, and the robot reachable on
`172.16.0.2` (default). Franka's own guide is the reference:
<https://frankaemika.github.io/docs/installation_linux.html>. The
[deoxys prerequisites page](https://zhuyifengzju.github.io/deoxys_docs/html/installation/system_prerequisite.html)
is a good condensed walkthrough of the same steps.

Check before continuing:

```bash
uname -v | grep -i preempt          # RT kernel
ulimit -r                            # >= 99 (rtprio)
ping -c1 172.16.0.2                  # FCI reachable
```

## 2. libfranka

FrankaTwin links against whatever `find_package(Franka)` finds. Three common setups:

**System install** (recommended):

```bash
# Ubuntu 22.04, FR3 system >= 5.7 -> libfranka 0.14 or 0.15
sudo apt install libfranka-dev        # or build from source and `cmake --install`
```

**Reuse a conda / deoxys build:**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/deoxys/build/libfranka
```

### Conda environment (libfranka from source)

No root on the NUC, or you want 0.13.x pinned alongside a newer system copy?
Build libfranka inside a conda env and install it into `$CONDA_PREFIX`. Pinning
0.13.3 also keeps you on the pre-Pinocchio dynamics model (see below).

```bash
conda create -n frankatwin -c conda-forge python=3.11 poco "eigen=3.4" \
    cmake cxx-compiler pkg-config make "sysroot_linux-64=2.28"
conda activate frankatwin

git clone --recursive --branch 0.13.3 https://github.com/frankaemika/libfranka.git
cmake -S libfranka -B libfranka/build -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build libfranka/build -j$(nproc) && cmake --install libfranka/build

ls $CONDA_PREFIX/lib/libfranka.so.*     # libfranka.so.0.13  libfranka.so.0.13.3
```

Then build FrankaTwin against it, with the env still active:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
cmake --build build -j$(nproc)
```

**Pin `sysroot_linux-64` in the `conda create` line — this is the part that
bites.** conda's compiler links against the *sysroot's* glibc, not the host's,
and the sysroot conda-forge picks by default (2.12) is older than what the rest
of the env needs:

| library | needs | symptom with the default 2.12 sysroot |
|---|---|---|
| `libstdc++.so.6` | `GLIBC_2.17` | `undefined reference to memcpy@GLIBC_2.14` / `secure_getenv@GLIBC_2.17`; CMake reports *"The C++ compiler is not able to compile a simple test program"* |
| `libPocoNet`, `libPocoFoundation` | `GLIBC_2.28` | `undefined reference to fcntl64@GLIBC_2.28` when linking `osc_shm` |

2.28 covers both. Note that 2.17 is *not* enough: it gets libfranka itself to
build (a shared library tolerates unresolved symbols in its own dependencies)
and only fails later, when the FrankaTwin executables are linked.

Any sysroot from 2.28 up to your host glibc works — check with `ldd --version`
(Ubuntu 20.04 → 2.31, 22.04 → 2.35). Choosing one *above* the host glibc
produces binaries that will not run.

Pin it in `conda create` rather than a follow-up `conda install`: one fresh
solve took 4.5 min on a test NUC, versus 11 min for `conda create` plus a
separate `conda install sysroot_linux-64=...`, because an incremental solve has
to keep 60+ already-installed packages consistent. If conda prints
`Error while loading conda entry point: conda-libmamba-solver`, it has fallen
back to the slow classic solver; `micromamba` does the same job in seconds:

```bash
micromamba install -p $CONDA_PREFIX -c conda-forge "sysroot_linux-64=2.28"
```

If a build directory survives from a failed attempt, delete it before retrying —
CMake caches "the compiler is broken" and will not retest it:

```bash
rm -rf libfranka/build build
```

### libfranka ≥ 0.14 needs Pinocchio

Starting with libfranka 0.14 (robot system ≥ 5.7.0) the dynamics model
(`mass`, `coriolis`, `gravity`, `zeroJacobian`) is computed with
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) instead of the old
internal model. Linking `Franka::Franka` then requires Pinocchio too, otherwise
you get undefined symbols like `pinocchio::computeJointJacobians` at link time.
`CMakeLists.txt` detects the libfranka version and adds
`find_package(pinocchio REQUIRED)` automatically; you only have to make it
findable:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="/path/to/libfranka;/path/to/pinocchio"
```

Install Pinocchio from apt (`robotpkg-py3*-pinocchio`), conda-forge (`pinocchio`),
or build it minimally with `-DBUILD_PYTHON_INTERFACE=OFF`. For libfranka < 0.14
nothing extra is needed.

### Conda caveats (only if `CONDA_PREFIX` is set)

`CMakeLists.txt` contains two workarounds that are active whenever a conda env is
activated at configure time:

- It prepends `$CONDA_PREFIX/lib` to the linker search path and RPATH so that a
  conda-built libfranka (which pulls conda's `libfmt` → newer `GLIBCXX`) links
  against conda's `libstdc++` instead of the system one.
- It links `libboost_filesystem.so.1.82.0` from conda directly if present, because
  binaries that run with file capabilities (`cap_sys_nice`) ignore
  `LD_LIBRARY_PATH` and would otherwise fail to load Pinocchio's parser deps.

If you use a system libfranka, configure with conda **deactivated** and both
workarounds stay off.

## 3. Build and install (NUC)

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
cmake -S . -B build && cmake --build build -j
ls build/osc_shm build/move_to build/read_current_q build/read_current_pose build/read_load

pip install -e .                                   # numpy, pyyaml, pyzmq
python -m pytest tests -q                          # shm ABI, config, excitation, cli
python -m frankatwin.doctor                                  # RT kernel, rtprio, binaries, libfranka, FCI link
```

`python -m frankatwin.doctor` prints one line per check with a hint on failure; fix the
`[XX]` lines before starting the daemon.

`osc_shm` asks for `SCHED_FIFO` priority 80 at startup. Either run the daemon
from a shell with `ulimit -r 99` (realtime group) or grant the capability once:

```bash
sudo setcap cap_sys_nice+ep build/osc_shm
```

If it can't get RT priority it prints `RT = non-RT` and continues; expect
occasional `communication_constraints_violation` under load in that mode.

## 4. Install (PC)

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
pip install -e ".[analysis]"      # + pandas/matplotlib for scripts/compare_*.py, plot_*.py
```

Edit `config/robot.yaml → network.nuc_host` to the NUC's address on the PC-facing
interface (the default `172.16.0.1` assumes the PC sits on the FCI subnet; use
the NUC's LAN IP otherwise). Ports 5555 (REQ/REP) and 5556 (PUB) must be open.

## 5. Configuration file

Both halves read the same `config/robot.yaml` from the checkout
(`pip install -e .` is the intended install mode). Override with `--config` on
any command or `FRANKATWIN_CONFIG=/path/to/local.yaml`. Keys are documented
inline in the file and in [Configuration](configuration.md).

## 6. First run

Terminal 1, **NUC**:

```bash
python -m frankatwin.doctor          # flags other FCI clients, missing binaries, unreachable FCI
python -m frankatwin.daemon -v
```

Expected banner:

```
[osc_shm] robot_ip = 172.16.0.2
[osc_shm] RT       = SCHED_FIFO
[osc_shm] tau_rate = 800.000000 Nm/s (slew limit)
[osc_shm] load     = none (using Desk-configured load)
[osc_shm] collision = torque 100 Nm, cartesian 100 N/Nm (reflex thresholds)
[osc_shm] starting 1 kHz loop. SIGINT to stop.
... daemon ready, awaiting commands
```

Terminal 2, **PC**:

```bash
python -m frankatwin.doctor          # daemon ping + state stream
python examples/move_to.py              # move_to -> init_q (home), then osc_shm resumes
python examples/cart_impedance.py          # 4 s, ±5 cm z-sine, prints tracking RMS
```

## 7. IsaacLab (only for sysid / replay)

The optimize / validate steps run in [IsaacLab](https://github.com/isaac-sim/IsaacLab)
≥ 2.3.0 (the dynamic/viscous joint-friction API landed in 2.3). Deploy the shipped
extension once into your checkout:

```bash
./isaaclab_sysid/install_into_isaaclab.sh /path/to/IsaacLab
conda activate <your isaaclab env> && pip install cmaes
```

This copies the `franka_sysid` task package, the `franka_mimic.usd` robot asset
and three scripts into the IsaacLab tree; no IsaacLab source edits. See
[sysid.md](sysid.md).
