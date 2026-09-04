"""Excitation trajectories for system identification.

* :mod:`frankatwin.excitation.multiband` -- SysID v3: two-band stationary
  sinusoids on x/y/z plus optional base-yaw / EE-roll sweeps.
* :mod:`frankatwin.excitation.chirp` -- SysID v4: 6-DOF linear chirp.

Both modules expose a ``build_*_trajectory`` function that evaluates the
reference on an arbitrary time grid (used by ``frankatwin-excite``) and a
``main()`` that writes a 1 kHz target CSV + sidecar (``frankatwin-gen-*``).
"""

from .chirp import (
    CHIRP_F0_DEFAULT,
    CHIRP_F1_DEFAULT,
    PHASE_OFFSETS,
    build_chirp_trajectory,
)
from .multiband import ORI_FREQS, POS_FREQS, build_multiband_trajectory

__all__ = [
    "CHIRP_F0_DEFAULT",
    "CHIRP_F1_DEFAULT",
    "PHASE_OFFSETS",
    "build_chirp_trajectory",
    "ORI_FREQS",
    "POS_FREQS",
    "build_multiband_trajectory",
]
