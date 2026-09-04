"""frankatwin: Python wrappers around the C++ 1 kHz Franka controller.

The heavy submodules (`local_controller`, `remote_client`, `daemon`) are
imported lazily so that `import frankatwin` does not pull in pyzmq or the
shm machinery unless the caller actually needs them.
"""

from __future__ import annotations

import importlib
from typing import Any

from frankatwin.config import RobotConfig, load_config

__version__ = "0.2.0"

__all__ = [
    "__version__",
    "LocalController",
    "FrankaTwinClient",
    "RobotConfig",
    "load_config",
]


def __getattr__(name: str) -> Any:
    if name == "LocalController":
        return importlib.import_module(".local_controller", __name__).LocalController
    if name == "FrankaTwinClient":
        return importlib.import_module(".remote_client", __name__).FrankaTwinClient
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
