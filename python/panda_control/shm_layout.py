"""POSIX shared-memory layout for panda_control Step 10.

This module is a faithful numpy.dtype mirror of `src/shm_layout.h`. The C++
side static-asserts the field offsets; this side enforces them via a unit
test (`tests/test_shm_layout.py`) using ctypes to cross-check. Keep both
in sync.

Concurrency model:
- Command (Python writer, C++ reader): seqlock with 64-bit `seq`. Even = stable.
- State (C++ writer, Python reader): SPSC ring buffer indexed by `state_head`.

All atomic operations are performed with explicit memory barriers
(`numpy.atomic`-like) via `_atomic_*` helpers below; numpy itself does not
provide atomics, so we rely on the GIL + write ordering of single-uint64
stores being atomic on x86_64 (they are when 8-byte aligned, which our
layout guarantees).
"""

from __future__ import annotations

import os
import struct
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Constants (must match src/shm_layout.h)
# ---------------------------------------------------------------------------
PANDA_SHM_MAGIC = 0x50414E44
PANDA_SHM_VERSION = 2  # v2: added state ee_linvel/ee_angvel
PANDA_SHM_STATE_FRAMES = 1024
PANDA_SHM_DEFAULT_NAME = "/panda_osc"

# ---------------------------------------------------------------------------
# numpy dtypes (must produce the same byte layout as the C++ structs)
# ---------------------------------------------------------------------------
HEADER_DTYPE = np.dtype(
    [
        ("magic", np.uint32),
        ("version", np.uint32),
        ("state_frames", np.uint32),
        ("reserved0", np.uint32),
        ("controller_pid", np.uint64),
        ("state_head", np.uint64),
    ],
    align=True,
)
assert HEADER_DTYPE.itemsize == 32, f"HEADER_DTYPE size {HEADER_DTYPE.itemsize} != 32"

COMMAND_DTYPE = np.dtype(
    [
        ("seq", np.uint64),
        ("target_pos", np.float64, (3,)),
        ("target_quat", np.float64, (4,)),  # wxyz
        ("kp_pos", np.float64),
        ("kp_ori", np.float64),
        ("kd_pos", np.float64),
        ("kd_ori", np.float64),
        ("error_delta_pos", np.float64),
        ("error_delta_rot", np.float64),
        ("enabled", np.uint32),
        ("reserved0", np.uint32),
    ],
    align=True,
)
assert COMMAND_DTYPE.itemsize == 120, (
    f"COMMAND_DTYPE size {COMMAND_DTYPE.itemsize} != 120"
)

STATE_FRAME_DTYPE = np.dtype(
    [
        ("seq", np.uint64),
        ("timestamp_s", np.float64),
        ("q", np.float64, (7,)),
        ("dq", np.float64, (7,)),
        ("ee_pos", np.float64, (3,)),
        ("ee_quat", np.float64, (4,)),  # wxyz
        ("tau", np.float64, (7,)),
        ("ee_linvel", np.float64, (3,)),  # base frame (m/s), v2+
        ("ee_angvel", np.float64, (3,)),  # base frame (rad/s), v2+
        ("reserved0", np.uint64),
        ("reserved1", np.uint64),
        ("reserved2", np.uint64),
        ("reserved3", np.uint64),
    ],
    align=True,
)
assert STATE_FRAME_DTYPE.itemsize == 320, (
    f"STATE_FRAME_DTYPE size {STATE_FRAME_DTYPE.itemsize} != 320"
)

SHM_TOTAL_BYTES = (
    HEADER_DTYPE.itemsize
    + COMMAND_DTYPE.itemsize
    + STATE_FRAME_DTYPE.itemsize * PANDA_SHM_STATE_FRAMES
)
assert SHM_TOTAL_BYTES == 32 + 120 + 1024 * 320

# Offset of each region within the shm buffer.
OFFSET_HEADER = 0
OFFSET_COMMAND = HEADER_DTYPE.itemsize
OFFSET_STATES = OFFSET_COMMAND + COMMAND_DTYPE.itemsize


# ---------------------------------------------------------------------------
# Single-uint64 atomic helpers via struct + memoryview.
# On x86_64 with natural 8-byte alignment, a single store/load of a uint64
# is atomic. We use struct.pack_into / unpack_from for clarity; the GIL
# serializes Python-level operations.
# ---------------------------------------------------------------------------
_U64 = struct.Struct("<Q")


def _atomic_store_u64(buf: memoryview, offset: int, value: int) -> None:
    _U64.pack_into(buf, offset, value & 0xFFFFFFFFFFFFFFFF)


def _atomic_load_u64(buf: memoryview, offset: int) -> int:
    return _U64.unpack_from(buf, offset)[0]


# ---------------------------------------------------------------------------
# View handle.
# ---------------------------------------------------------------------------
@dataclass
class PandaShmView:
    """Read/write views over an open POSIX shared-memory segment.

    Lifetime: caller owns the underlying `shared_memory.SharedMemory` object
    (see `SharedMemoryAccess`). All numpy views are zero-copy aliases of the
    same buffer.
    """

    raw: memoryview
    header: np.ndarray
    command: np.ndarray
    states: np.ndarray

    # ----- header -----
    @property
    def magic(self) -> int:
        return int(self.header["magic"][0])

    @property
    def version(self) -> int:
        return int(self.header["version"][0])

    @property
    def state_frames(self) -> int:
        return int(self.header["state_frames"][0])

    @property
    def controller_pid(self) -> int:
        return _atomic_load_u64(self.raw, OFFSET_HEADER + 16)

    @controller_pid.setter
    def controller_pid(self, value: int) -> None:
        _atomic_store_u64(self.raw, OFFSET_HEADER + 16, value)

    @property
    def state_head(self) -> int:
        return _atomic_load_u64(self.raw, OFFSET_HEADER + 24)

    # ----- command (Python writer, C++ reader) -----
    def write_command(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        kp_pos: float,
        kp_ori: float,
        kd_pos: float,
        kd_ori: float,
        error_delta_pos: float,
        error_delta_rot: float,
        enabled: bool,
    ) -> None:
        """Atomically publish a new command. Uses the seqlock protocol.

        Caller must ensure target_pos has shape (3,) and target_quat has
        shape (4,) in wxyz convention with unit norm.
        """
        seq_offset = OFFSET_COMMAND  # `seq` is the first field
        cur = _atomic_load_u64(self.raw, seq_offset)
        odd = cur + 1
        _atomic_store_u64(self.raw, seq_offset, odd)

        # Plain stores into the dtype view. numpy assignment goes through the
        # same buffer; ordering is guaranteed by the surrounding atomic stores
        # plus the GIL.
        c = self.command
        c["target_pos"][0] = target_pos
        c["target_quat"][0] = target_quat
        c["kp_pos"][0] = kp_pos
        c["kp_ori"][0] = kp_ori
        c["kd_pos"][0] = kd_pos
        c["kd_ori"][0] = kd_ori
        c["error_delta_pos"][0] = error_delta_pos
        c["error_delta_rot"][0] = error_delta_rot
        c["enabled"][0] = np.uint32(1 if enabled else 0)

        _atomic_store_u64(self.raw, seq_offset, odd + 1)

    def read_command(self) -> np.ndarray:
        """Read the current command (seqlock-safe). Returns a copy."""
        seq_offset = OFFSET_COMMAND
        while True:
            s1 = _atomic_load_u64(self.raw, seq_offset)
            if s1 & 1:
                continue
            snapshot = self.command.copy()
            s2 = _atomic_load_u64(self.raw, seq_offset)
            if s1 == s2:
                return snapshot

    # ----- state (Python reader) -----
    def latest_state(self) -> Optional[np.ndarray]:
        """Return a copy of the most recently published state frame.

        Returns None if the producer has not written any frame yet.
        """
        head = self.state_head
        if head == 0:
            return None
        idx = head % PANDA_SHM_STATE_FRAMES
        snapshot = self.states[idx].copy()
        head2 = self.state_head
        if head2 - head >= PANDA_SHM_STATE_FRAMES:
            # Producer wrapped during our read; frame is suspect.
            return None
        return snapshot

    def last_k_states(self, k: int) -> np.ndarray:
        """Return up to k most recent state frames, oldest first. Best-effort:
        if the producer wrapped during the copy, only the consistent suffix
        is returned. Returns an empty array if no frames exist.
        """
        if k <= 0:
            return np.empty(0, dtype=STATE_FRAME_DTYPE)
        head = self.state_head
        if head == 0:
            return np.empty(0, dtype=STATE_FRAME_DTYPE)
        k = min(k, PANDA_SHM_STATE_FRAMES, head)
        idxs = [(head - k + 1 + i) % PANDA_SHM_STATE_FRAMES for i in range(k)]
        frames = np.stack([self.states[i].copy() for i in idxs])
        # Drop any prefix whose seq is stale (producer overwrote it mid-read).
        valid = frames["seq"] > head - PANDA_SHM_STATE_FRAMES
        return frames[valid]


# ---------------------------------------------------------------------------
# Shared-memory open / create.
# ---------------------------------------------------------------------------
class SharedMemoryAccess:
    """RAII wrapper over multiprocessing.shared_memory.SharedMemory.

    On Linux this maps to `/dev/shm/<name>` (the leading '/' in the C-style
    POSIX shm name is stripped by SharedMemory; pass either `/panda_osc` or
    `panda_osc` and we normalize).
    """

    def __init__(
        self,
        name: str = PANDA_SHM_DEFAULT_NAME,
        create: bool = False,
    ) -> None:
        norm = name.lstrip("/")
        if create:
            try:
                # Reset any stale leftover; ignore if absent.
                _unlink_quietly(norm)
            except FileNotFoundError:
                pass
            self.shm = shared_memory.SharedMemory(
                name=norm, create=True, size=SHM_TOTAL_BYTES
            )
            # Zero the buffer and stamp the header.
            buf = self.shm.buf
            buf[:] = b"\x00" * SHM_TOTAL_BYTES
            header_view = np.frombuffer(buf, dtype=HEADER_DTYPE, count=1)
            header_view[0]["magic"] = PANDA_SHM_MAGIC
            header_view[0]["version"] = PANDA_SHM_VERSION
            header_view[0]["state_frames"] = PANDA_SHM_STATE_FRAMES
        else:
            self.shm = shared_memory.SharedMemory(name=norm, create=False)
            if self.shm.size != SHM_TOTAL_BYTES:
                size = self.shm.size
                self.shm.close()
                raise RuntimeError(
                    f"shm '{norm}' has unexpected size {size} (expected {SHM_TOTAL_BYTES})"
                )

        self._created = create
        self.view = _build_view(self.shm.buf)
        if self.view.magic != PANDA_SHM_MAGIC:
            raise RuntimeError(
                f"shm '{norm}' magic mismatch: got 0x{self.view.magic:08x}"
            )
        if self.view.version != PANDA_SHM_VERSION:
            raise RuntimeError(
                f"shm '{norm}' version mismatch: got {self.view.version}, "
                f"expected {PANDA_SHM_VERSION}"
            )

    def close(self, unlink: Optional[bool] = None) -> None:
        if unlink is None:
            unlink = self._created
        try:
            self.shm.close()
        except Exception:
            pass
        if unlink:
            try:
                self.shm.unlink()
            except (FileNotFoundError, Exception):
                pass

    def __enter__(self) -> "SharedMemoryAccess":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


def _build_view(buf: memoryview) -> PandaShmView:
    header = np.frombuffer(buf, dtype=HEADER_DTYPE, count=1, offset=OFFSET_HEADER)
    command = np.frombuffer(
        buf, dtype=COMMAND_DTYPE, count=1, offset=OFFSET_COMMAND
    )
    states = np.frombuffer(
        buf,
        dtype=STATE_FRAME_DTYPE,
        count=PANDA_SHM_STATE_FRAMES,
        offset=OFFSET_STATES,
    )
    return PandaShmView(raw=buf, header=header, command=command, states=states)


def _unlink_quietly(name: str) -> None:
    """Remove a stale shm segment if it exists. No-op otherwise."""
    path = f"/dev/shm/{name}"
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass


# Convenience tuple for tests / introspection.
def layout_offsets() -> Tuple[int, int, int, int]:
    """Return (header_size, command_size, state_frame_size, total_size)."""
    return (
        HEADER_DTYPE.itemsize,
        COMMAND_DTYPE.itemsize,
        STATE_FRAME_DTYPE.itemsize,
        SHM_TOTAL_BYTES,
    )
