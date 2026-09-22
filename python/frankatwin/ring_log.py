"""1 kHz run log from the shm state ring, with tick-exact setpoint events.

Why this exists: ``examples/cart_impedance.py --log`` records one row per
setpoint tick (50 Hz) and fills it with the newest frame of the daemon's
100 Hz ZMQ stream, so each row's state is 0-10 ms older than its nominal
``t_s``. The sim replay then scores at those nominal instants. That offset is
larger than the motor delay the fit tries to identify (0-4 ms), and it
throws away 95 % of the frames the controller produced.

The controller writes one ``ShmStateFrame`` per 1 kHz tick into a ring of
``FRANKATWIN_SHM_STATE_FRAMES`` (1024) slots (``src/shm_layout.h``,
``state_publish``): frame ``seq`` lives in slot ``seq % 1024`` and
``header.state_head`` is the newest seq. A reader polling every 10 ms sees
about ten new frames per poll and has ~1 s of slack before the producer laps
it, so a plain Python thread can copy every frame losslessly
(:class:`RingDrainer`).

Setpoints are stamped by whoever writes them (``LocalController.set_ee_target``
reads ``state_head`` right after the seqlock write and hands it to
:meth:`RunRecorder.record_target`). Within one tick ``osc_shm`` snapshots the
command *before* it publishes that tick's frame, so a target written when
``state_head == H`` is used by tick H+1 at the latest -- by tick H itself only
if the write raced ahead of tick H's snapshot. :func:`merge_run` treats target
k as active from frame ``head_k + 1``; the residual uncertainty is one tick.

The merged CSV has the columns of ``cart_impedance.py --log`` in the same
order (``sysid_franka_osc.py`` and ``replay_python_csv_sim.py`` read it
unchanged), followed by ``seq`` and ``target_idx``. Differences from the
50 Hz log: ``t_s`` is ``(seq - seq_first) / 1000`` (tick count, not wall
clock); ``period_ms`` is the difference of the producer's own timestamps;
``dx_des_*`` is 0 because the command block carries no velocity feedforward
(``osc_shm`` never sees one -- the 50 Hz log's ``dx_des`` is the analytic
reference, written for plots only).

Where the files land: the recorder has to run next to the shm segment (the
NUC), the files are wanted where the run was driven from (the PC). So a session
can stay in memory (:class:`RunRecorder` without a ``path``): ``stop`` merges
into :attr:`RunRecorder.table` / :attr:`RunRecorder.targets_table`, the daemon
hands those rows out in chunks (op ``log_fetch``) and
``FrankaTwinClient.log_save`` writes them with :func:`write_run` on the PC.
With a ``path`` (``scripts/shm_log.py``, a ``LocalController`` used directly)
``stop`` writes on this machine.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from frankatwin.shm_layout import FRANKATWIN_SHM_STATE_FRAMES, STATE_FRAME_DTYPE

logger = logging.getLogger(__name__)

TICK_S = 1.0e-3
DEFAULT_POLL_HZ = 100.0

# Column order of examples/cart_impedance.py `_write_csv`. Kept verbatim so a
# ring log is a drop-in replacement for the 50 Hz log downstream.
CART_IMPEDANCE_COLUMNS: List[str] = (
    ["t_s", "period_ms"]
    + [f"q{i}" for i in range(1, 8)]
    + [f"dq{i}" for i in range(1, 8)]
    + ["x_x", "x_y", "x_z"]
    + ["quat_x", "quat_y", "quat_z", "quat_w"]
    + ["x_des_x", "x_des_y", "x_des_z"]
    + ["dx_des_x", "dx_des_y", "dx_des_z"]
    + ["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]
    + [f"tau{i}" for i in range(1, 8)]
    + [f"tau_J{i}" for i in range(1, 8)]
)
MERGED_COLUMNS: List[str] = CART_IMPEDANCE_COLUMNS + ["seq", "target_idx"]
TARGETS_COLUMNS: List[str] = [
    "target_idx", "head", "seq_effective",
    "x_des_x", "x_des_y", "x_des_z",
    "quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w",
]
# The two tables of a finished session, by the name `log_fetch` knows them under.
TABLES: Dict[str, List[str]] = {"merged": MERGED_COLUMNS, "targets": TARGETS_COLUMNS}


def _wxyz_to_xyzw(q: np.ndarray) -> np.ndarray:
    return np.asarray(q, dtype=np.float64)[..., [1, 2, 3, 0]]


# ---------------------------------------------------------------------------
# Ring drainer
# ---------------------------------------------------------------------------
class RingDrainer:
    """Copies every new frame out of the state ring across successive polls.

    ``poll()`` reads ``state_head``, copies the frames with seq in
    ``(last_seq, head]`` oldest-first and checks each slot's ``seq``: a slot
    the producer overwrote while we copied carries a newer seq and is dropped
    (same guard as ``ShmView.last_k_states``). If more than ``ring_size``
    frames arrived since the previous poll, the oldest are gone for good;
    ``gaps`` records every ``(first_missing, last_missing)`` range and
    ``dropped`` counts the frames. A ``head`` below ``last_seq`` means the
    producer restarted with a re-initialised header (``osc_shm --init-shm``);
    the drainer re-anchors and counts it in ``resets``.

    ``view`` needs ``state_head`` (int) and ``states`` (structured array of
    ``ring_size`` frames) -- a ``ShmView`` or anything shaped like one.
    """

    def __init__(self, view, ring_size: int = FRANKATWIN_SHM_STATE_FRAMES) -> None:
        self._view = view
        self._ring = int(ring_size)
        self.last_seq: int = 0
        self.gaps: List[Tuple[int, int]] = []
        self.dropped: int = 0
        self.resets: int = 0
        self.num_frames: int = 0
        self._chunks: List[np.ndarray] = []

    def poll(self) -> int:
        """Copy the frames published since the last poll. Returns how many."""
        head = int(self._view.state_head)
        if head == 0:
            return 0
        if head < self.last_seq:
            self.resets += 1
            self.last_seq = 0
        first = self.last_seq + 1
        if head - first + 1 > self._ring:
            lost_to = head - self._ring
            self.gaps.append((first, lost_to))
            self.dropped += lost_to - first + 1
            first = lost_to + 1
        if first > head:
            return 0
        seqs = np.arange(first, head + 1, dtype=np.uint64)
        idx = (seqs % np.uint64(self._ring)).astype(np.intp)
        frames = self._view.states[idx].copy()
        ok = frames["seq"] == seqs
        if not np.all(ok):
            # The producer reused some of these slots while we were copying.
            # The overwritten ones are the oldest; their newer contents will be
            # picked up on the next poll under their own seq.
            bad = seqs[~ok].astype(np.int64)
            self.gaps.append((int(bad.min()), int(bad.max())))
            self.dropped += int(bad.size)
            frames = frames[ok]
        self.last_seq = head
        if frames.size:
            self._chunks.append(frames)
            self.num_frames += int(frames.size)
        return int(frames.size)

    def frames(self) -> np.ndarray:
        """All frames copied so far, oldest first."""
        if not self._chunks:
            return np.empty(0, dtype=STATE_FRAME_DTYPE)
        return np.concatenate(self._chunks)

    def run(self, stop_evt: threading.Event, poll_hz: float = DEFAULT_POLL_HZ) -> None:
        """Poll at ``poll_hz`` until ``stop_evt`` is set, then poll once more."""
        period = 1.0 / float(poll_hz)
        next_t = time.monotonic()
        while not stop_evt.is_set():
            self.poll()
            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                stop_evt.wait(sleep_for)
            else:
                next_t = time.monotonic()
        self.poll()


# ---------------------------------------------------------------------------
# Setpoint events + merge
# ---------------------------------------------------------------------------
@dataclass
class TargetEvent:
    """A setpoint write, stamped with the ``state_head`` seen right after it."""

    idx: int
    head: int
    pos: np.ndarray        # (3,) base frame [m]
    quat_wxyz: np.ndarray  # (4,) unit

    @property
    def seq_effective(self) -> int:
        """First frame whose torque was computed under this target (see module doc)."""
        return self.head + 1


def merge_run(
    frames: np.ndarray,
    targets: Sequence[TargetEvent],
    start_pos: np.ndarray,
    start_quat_wxyz: np.ndarray,
) -> np.ndarray:
    """Build the merged table (rows = frames, columns = ``MERGED_COLUMNS``).

    ``start_pos`` / ``start_quat_wxyz`` is the target the command block held
    when the session started; frames before the first event carry it with
    ``target_idx = -1``. Frames are sorted by ``seq``; events by ``head``. When
    two events share a head (two writes inside one tick) the later one wins,
    which is what the controller saw too.
    """
    n = int(frames.shape[0])
    if n == 0:
        return np.empty((0, len(MERGED_COLUMNS)), dtype=np.float64)
    order = np.argsort(frames["seq"], kind="stable")
    frames = frames[order]
    seq = frames["seq"].astype(np.int64)

    ev = sorted(targets, key=lambda e: (e.head, e.idx))
    eff = np.array([e.seq_effective for e in ev], dtype=np.int64)
    k = np.searchsorted(eff, seq, side="right") - 1          # -1 -> start snapshot
    pos_table = np.vstack([np.asarray(start_pos, dtype=np.float64).reshape(1, 3)]
                          + [np.asarray(e.pos, dtype=np.float64).reshape(1, 3) for e in ev])
    quat_table = np.vstack([np.asarray(start_quat_wxyz, dtype=np.float64).reshape(1, 4)]
                           + [np.asarray(e.quat_wxyz, dtype=np.float64).reshape(1, 4) for e in ev])
    idx_table = np.array([-1] + [e.idx for e in ev], dtype=np.int64)
    x_des = pos_table[k + 1]
    quat_des_xyzw = _wxyz_to_xyzw(quat_table[k + 1])
    target_idx = idx_table[k + 1]

    t_s = (seq - seq[0]).astype(np.float64) * TICK_S
    period_ms = np.zeros(n, dtype=np.float64)
    period_ms[1:] = np.diff(frames["timestamp_s"]) * 1.0e3

    return np.column_stack((
        t_s,
        period_ms,
        frames["q"],
        frames["dq"],
        frames["ee_pos"],
        _wxyz_to_xyzw(frames["ee_quat"]),
        x_des,
        np.zeros((n, 3), dtype=np.float64),      # dx_des: no feedforward in the command block
        quat_des_xyzw,
        frames["tau"],
        frames["tau_J"],
        seq.astype(np.float64),
        target_idx.astype(np.float64),
    ))


def write_merged_csv(path: Path, table: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fmt = ["%.9f"] * len(CART_IMPEDANCE_COLUMNS) + ["%d", "%d"]
    np.savetxt(path, table, delimiter=",", header=",".join(MERGED_COLUMNS), comments="", fmt=fmt)


def targets_table(targets: Sequence[TargetEvent]) -> np.ndarray:
    """The setpoint events as a table (rows = events, columns = ``TARGETS_COLUMNS``)."""
    return np.array(
        [[e.idx, e.head, e.seq_effective, *e.pos, *_wxyz_to_xyzw(e.quat_wxyz)] for e in targets],
        dtype=np.float64,
    ).reshape(-1, len(TARGETS_COLUMNS))


def write_targets_csv(path: Path, targets) -> None:
    """``targets``: the events, or their table (:func:`targets_table`)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = targets if isinstance(targets, np.ndarray) else targets_table(targets)
    fmt = ["%d", "%d", "%d"] + ["%.9f"] * 7
    np.savetxt(path, rows, delimiter=",", header=",".join(TARGETS_COLUMNS), comments="", fmt=fmt)


def targets_path_for(path: Path) -> Path:
    path = Path(path)
    return path.with_name(path.stem + "_targets" + path.suffix)


def write_run(path, table: np.ndarray, targets) -> Dict[str, str]:
    """Write the merged CSV at ``path`` and ``<stem>_targets<suffix>`` next to it.

    Returns ``{"path", "targets_path"}``. Shared by :meth:`RunRecorder.save`
    and ``FrankaTwinClient.log_save`` so both machines write the same file.
    """
    path = Path(path).expanduser()
    tpath = targets_path_for(path)
    write_merged_csv(path, table)
    write_targets_csv(tpath, targets)
    return {"path": str(path), "targets_path": str(tpath)}


# ---------------------------------------------------------------------------
# Recording session
# ---------------------------------------------------------------------------
class RunRecorder:
    """One recording session: a drainer thread plus the setpoint events.

    Frames published before :meth:`start` are skipped; the target in force at
    that moment is read from the command block so every row has a setpoint.
    :meth:`record_target` is called by the setpoint writer while the session
    runs. :meth:`stop` joins the thread, drains once more, merges into
    :attr:`table` / :attr:`targets_table` and returns a summary dict; with a
    ``path`` it also writes ``path`` and ``<stem>_targets<suffix>`` on this
    machine (:meth:`save`). Without a ``path`` the session stays in memory, for
    whoever holds the recorder to read the tables or :meth:`save` them later.
    Without :meth:`start` (no thread) the caller drives :meth:`poll` itself.
    """

    def __init__(self, view, path=None, *, poll_hz: float = DEFAULT_POLL_HZ) -> None:
        self.view = view
        self.path: Optional[Path] = None if path is None else Path(path).expanduser()
        self.poll_hz = float(poll_hz)
        self.drainer = RingDrainer(view)
        self.targets: List[TargetEvent] = []
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        cmd = view.read_command()
        self.start_pos = np.array(cmd["target_pos"][0], dtype=np.float64)
        self.start_quat_wxyz = np.array(cmd["target_quat"][0], dtype=np.float64)
        self.seq_start = int(view.state_head)
        self.drainer.last_seq = self.seq_start
        self._stopped = False
        # Set by stop(): the merged rows, the setpoint rows, the summary.
        self.table: Optional[np.ndarray] = None
        self.targets_table: Optional[np.ndarray] = None
        self.result: Dict[str, Any] = {}

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("recorder already started")
        self._thread = threading.Thread(
            target=self.drainer.run, args=(self._stop_evt, self.poll_hz),
            name="frankatwin_ring_log", daemon=True,
        )
        self._thread.start()

    def poll(self) -> int:
        return self.drainer.poll()

    def record_target(self, head: int, pos: np.ndarray, quat_wxyz: np.ndarray) -> TargetEvent:
        with self._lock:
            ev = TargetEvent(
                idx=len(self.targets),
                head=int(head),
                pos=np.array(pos, dtype=np.float64).reshape(3),
                quat_wxyz=np.array(quat_wxyz, dtype=np.float64).reshape(4),
            )
            self.targets.append(ev)
        return ev

    def stop(self, *, save: bool = True) -> Dict[str, Any]:
        """Stop and merge. ``save`` writes the files when a ``path`` was given."""
        if self._stopped:
            raise RuntimeError("recorder already stopped")
        self._stopped = True
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        else:
            self.drainer.poll()
        frames = self.drainer.frames()
        with self._lock:
            targets = list(self.targets)
        self.table = merge_run(frames, targets, self.start_pos, self.start_quat_wxyz)
        self.targets_table = targets_table(targets)
        self.result = self.summary(frames, targets)
        if save and self.path is not None:
            self.save()
        return dict(self.result)

    def save(self, path=None) -> Dict[str, str]:
        """Write the finished session on this machine, at ``path`` (default: the
        constructor's). Returns ``{"path", "targets_path"}``, also put in the summary."""
        if self.table is None or self.targets_table is None:
            raise RuntimeError("recorder still running: stop() first")
        path = self.path if path is None else path
        if path is None:
            raise ValueError("no path to save the ring log at (none given here or at construction)")
        written = write_run(path, self.table, self.targets_table)
        self.result.update(written)
        return written

    def summary(self, frames: np.ndarray, targets: Sequence[TargetEvent]) -> Dict[str, Any]:
        n = int(frames.shape[0])
        seq_first = int(frames["seq"].min()) if n else 0
        seq_last = int(frames["seq"].max()) if n else 0
        return {
            # Filled in by save(); None while the session only lives in memory.
            "path": None,
            "targets_path": None,
            "num_frames": n,
            "seq_start": self.seq_start,
            "seq_first": seq_first,
            "seq_last": seq_last,
            "duration_s": (seq_last - seq_first + 1) * TICK_S if n else 0.0,
            "num_targets": len(targets),
            "gaps": [[int(a), int(b)] for a, b in self.drainer.gaps],
            "dropped_frames": int(self.drainer.dropped),
            "resets": int(self.drainer.resets),
            "poll_hz": self.poll_hz,
        }
