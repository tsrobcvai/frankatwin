"""panda_control: Python wrappers around the C++ 1 kHz Franka controller.

The heavy submodules (`local_controller`, `remote_client`, `daemon`) are
imported lazily so that `import panda_control` does not pull in pyzmq or the
shm machinery unless the caller actually needs them.
"""

from __future__ import annotations

import importlib
from typing import Any

from panda_control.config import RobotConfig, load_config

__all__ = [
    "LocalPandaController",
    "RemotePandaClient",
    "RobotConfig",
    "load_config",
]


def __getattr__(name: str) -> Any:
    if name == "LocalPandaController":
        return importlib.import_module(".local_controller", __name__).LocalPandaController
    if name == "RemotePandaClient":
        return importlib.import_module(".remote_client", __name__).RemotePandaClient
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
