"""Layout pinning test: numpy.dtype offsets MUST match the C++ struct layout.

How it works:
  1. Compile tests/dump_shm_offsets.cpp with the system g++ (no libfranka deps).
  2. Run it, parse the lines `size:<Type>:<N>` and `off:<Type>.<Member>:<N>`.
  3. Cross-reference against the same offsets computed from
     frankatwin.shm_layout's numpy dtypes.

This guards against silent ABI drift if anyone reorders, renames, or repads
a field in either side.
"""

from __future__ import annotations

import os
import pathlib
import shutil
import subprocess
from typing import Dict, Tuple

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TESTS = ROOT / "tests"

# Importable by either `pytest tests/` from the repo root or by the package.
import sys
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.shm_layout import (  # noqa: E402
    COMMAND_DTYPE,
    HEADER_DTYPE,
    FRANKATWIN_SHM_STATE_FRAMES,
    SHM_TOTAL_BYTES,
    STATE_FRAME_DTYPE,
)


@pytest.fixture(scope="module")
def cpp_offsets(tmp_path_factory) -> Tuple[Dict[str, int], Dict[str, int]]:
    """Compile and run dump_shm_offsets, return (sizes, offsets) dicts."""
    cxx = shutil.which("g++") or shutil.which("c++")
    if cxx is None:
        pytest.skip("g++/c++ not available")
    src = TESTS / "dump_shm_offsets.cpp"
    out = tmp_path_factory.mktemp("shm_offsets") / "dump"
    cmd = [
        cxx,
        "-std=c++17",
        "-O0",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-I",
        str(SRC),
        str(src),
        "-o",
        str(out),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    assert proc.returncode == 0, (
        f"compile failed\nstdout: {proc.stdout}\nstderr: {proc.stderr}"
    )
    proc = subprocess.run([str(out)], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    sizes: Dict[str, int] = {}
    offsets: Dict[str, int] = {}
    for line in proc.stdout.splitlines():
        kind, key, val = line.split(":", 2)
        if kind == "size":
            sizes[key] = int(val)
        elif kind == "off":
            offsets[key] = int(val)
    return sizes, offsets


def _np_offset(dtype, field: str) -> int:
    return dtype.fields[field][1]


def test_sizes(cpp_offsets):
    sizes, _ = cpp_offsets
    assert sizes["ShmHeader"] == HEADER_DTYPE.itemsize == 32
    assert sizes["ShmCommand"] == COMMAND_DTYPE.itemsize == 120
    assert sizes["ShmStateFrame"] == STATE_FRAME_DTYPE.itemsize == 384
    assert sizes["ShmSegment"] == SHM_TOTAL_BYTES
    assert SHM_TOTAL_BYTES == 32 + 120 + 1024 * 384
    assert FRANKATWIN_SHM_STATE_FRAMES == 1024


def test_header_offsets(cpp_offsets):
    _, off = cpp_offsets
    assert off["ShmHeader.magic"] == _np_offset(HEADER_DTYPE, "magic") == 0
    assert off["ShmHeader.version"] == _np_offset(HEADER_DTYPE, "version") == 4
    assert off["ShmHeader.state_frames"] == _np_offset(HEADER_DTYPE, "state_frames") == 8
    assert off["ShmHeader.controller_pid"] == _np_offset(HEADER_DTYPE, "controller_pid") == 16
    assert off["ShmHeader.state_head"] == _np_offset(HEADER_DTYPE, "state_head") == 24


def test_command_offsets(cpp_offsets):
    _, off = cpp_offsets
    pairs = [
        ("seq", 0),
        ("target_pos", 8),
        ("target_quat", 32),
        ("kp_pos", 64),
        ("kp_ori", 72),
        ("kd_pos", 80),
        ("kd_ori", 88),
        ("error_delta_pos", 96),
        ("error_delta_rot", 104),
        ("enabled", 112),
    ]
    for field, expected in pairs:
        cpp = off[f"ShmCommand.{field}"]
        np_off = _np_offset(COMMAND_DTYPE, field)
        assert cpp == np_off == expected, (
            f"command.{field}: cpp={cpp} numpy={np_off} expected={expected}"
        )


def test_state_offsets(cpp_offsets):
    _, off = cpp_offsets
    pairs = [
        ("seq", 0),
        ("timestamp_s", 8),
        ("q", 16),
        ("dq", 72),
        ("ee_pos", 128),
        ("ee_quat", 152),
        ("tau", 184),
        ("ee_linvel", 240),
        ("ee_angvel", 264),
        ("tau_J", 288),
    ]
    for field, expected in pairs:
        cpp = off[f"ShmStateFrame.{field}"]
        np_off = _np_offset(STATE_FRAME_DTYPE, field)
        assert cpp == np_off == expected, (
            f"state.{field}: cpp={cpp} numpy={np_off} expected={expected}"
        )
