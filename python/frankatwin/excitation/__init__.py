"""Excitation trajectories for system identification.

* :mod:`frankatwin.excitation.multiband` -- stationary sinusoids: 6-DOF with
  tilt, three bands up to 2 Hz, fades out. The held-out validation run for a
  chirp fit (``MULTIBAND_PROFILES["heldout"]``).
* :mod:`frankatwin.excitation.chirp` -- 6-DOF linear chirp, two
  bands (``low`` 0.1-0.7 Hz, ``high`` 0.7-3 Hz with a (0.7/f)^0.5 amplitude
  taper and a 1 s fade-out).

Both modules expose a ``build_*_trajectory`` function that evaluates the
reference on an arbitrary time grid. ``examples/cart_impedance.py`` drives the
robot with it; ``scripts/gen_*_traj.py`` write it out as a 1 kHz CSV + sidecar.
"""

from .chirp import (
    CHIRP_BANDS,
    CHIRP_F0_DEFAULT,
    CHIRP_F1_DEFAULT,
    PHASE_OFFSETS,
    build_chirp_trajectory,
)
from .multiband import (
    HELDOUT_FREQS,
    MULTIBAND_PROFILES,
    build_multiband_trajectory,
    multiband_tones,
)

__all__ = [
    "CHIRP_BANDS",
    "CHIRP_F0_DEFAULT",
    "CHIRP_F1_DEFAULT",
    "PHASE_OFFSETS",
    "build_chirp_trajectory",
    "HELDOUT_FREQS",
    "MULTIBAND_PROFILES",
    "build_multiband_trajectory",
    "multiband_tones",
]
