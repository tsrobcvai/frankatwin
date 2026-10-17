"""Versioned measurement bundles and common derived channels."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .math import rotation, pose_matrix, support_wrench
from .protocol import Protocol

SCHEMA_VERSION = 1
CONVENTION = "robot_on_payload; axes=O(base); moment_origin=K; SI"
AXES = ["fx", "fy", "fz", "tx", "ty", "tz"]


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def save_bundle(directory, arrays, metadata):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(directory / "samples.npz", **arrays)
    metadata = dict(
        metadata, schema_version=SCHEMA_VERSION, wrench_convention=CONVENTION
    )
    write_json(directory / "metadata.json", metadata)
    columns = {
        "t_s": arrays["t_s"],
        "phase": arrays["phase"],
        "settled": arrays["settled"].astype(int),
    }
    for key in ("wrench_O_K", "wrench_K_K", "gravity_O_K", "gravity_K_K"):
        for i, axis in enumerate(AXES):
            columns[f"{key}_{axis}"] = arrays[key][:, i]
    for i in range(7):
        columns[f"q{i+1}"] = arrays["q"][:, i]
    np.savetxt(
        directory / "samples.csv",
        np.column_stack(list(columns.values())),
        delimiter=",",
        header=",".join(columns),
        comments="",
        fmt="%.9g",
    )


def load_bundle(directory):
    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text())
    if (
        metadata.get("schema_version") != SCHEMA_VERSION
        or metadata.get("wrench_convention") != CONVENTION
    ):
        raise ValueError("incompatible measurement schema or wrench convention")
    with np.load(directory / "samples.npz", allow_pickle=False) as file:
        arrays = {key: file[key] for key in file.files}
    t = arrays["t_s"]
    if (
        t.ndim != 1
        or len(t) < 3
        or not np.all(np.isfinite(t))
        or np.any(np.diff(t) <= 0)
    ):
        raise ValueError("invalid timestamps")
    protocol = Protocol(**metadata["protocol"]).validate()
    if protocol.digest != metadata["protocol_sha256"]:
        raise ValueError("protocol hash mismatch")
    for key, width in (
        ("q", 7),
        ("ee_pos", 3),
        ("ee_quat", 4),
        ("wrench_O_K", 6),
        ("wrench_K_K", 6),
        ("gravity_O_K", 6),
        ("gravity_K_K", 6),
    ):
        if arrays[key].shape != (len(t), width):
            raise ValueError(f"invalid shape: {key}")
        if not np.all(np.isfinite(arrays[key])):
            raise ValueError(f"nonfinite {key}")
    if not np.allclose(np.linalg.norm(arrays["ee_quat"], axis=1), 1, atol=1e-5):
        raise ValueError("EE quaternions must be normalized")
    for key in ("phase", "settled"):
        if arrays[key].shape != t.shape:
            raise ValueError(f"invalid shape: {key}")
    return arrays, metadata


def derive_frames(arrays, protocol, ee_t_k):
    ee = pose_matrix(arrays["ee_pos"], arrays["ee_quat"])
    ok = ee @ ee_t_k
    arrays["k_pos"] = ok[:, :3, 3]
    arrays["k_rot"] = ok[:, :3, :3]
    com = arrays["ee_pos"] + np.einsum(
        "nij,j->ni", rotation(arrays["ee_quat"]), protocol.ee_com_m
    )
    gravity = support_wrench(protocol.mass_kg, com, arrays["k_pos"])
    arrays["gravity_O_K"] = gravity
    arrays["gravity_K_K"] = np.concatenate(
        [
            np.einsum("nji,nj->ni", arrays["k_rot"], gravity[:, :3]),
            np.einsum("nji,nj->ni", arrays["k_rot"], gravity[:, 3:]),
        ],
        axis=1,
    )
    w = arrays["wrench_O_K"]
    arrays["wrench_K_K"] = np.concatenate(
        [
            np.einsum("nji,nj->ni", arrays["k_rot"], w[:, :3]),
            np.einsum("nji,nj->ni", arrays["k_rot"], w[:, 3:]),
        ],
        axis=1,
    )
