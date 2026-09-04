# Installation

FrankaTwin runs as two halves:

| where | what | needs |
|---|---|---|
| **NUC** (real-time PC wired to the robot) | `osc_shm`, `move_to` (C++), `frankatwin.daemon` (Python) | RT kernel, libfranka, Eigen3, CMake ≥ 3.10, Python ≥ 3.9 |
| **PC** (your workstation) | `FrankaTwinClient` + examples/scripts (Python) | Python ≥ 3.9, network route to the NUC |

If you run everything on one machine, install both halves there.

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

FrankaTwin links against whatever `find_package(Franka)` finds. Two common setups:

**System install** (recommended):

```bash
# Ubuntu 22.04, FR3 system >= 5.7 -> libfranka 0.14 or 0.15
sudo apt install libfranka-dev        # or build from source and `cmake --install`
```

**Reuse a conda / deoxys build:**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/deoxys/build/libfranka
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
inline in the file and in [usage.md](usage.md#configuration-reference).

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
