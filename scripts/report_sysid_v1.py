#!/usr/bin/env python3
"""Aggregate held-out + ablation metrics into markdown report and bar plot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


POS = ["x_x", "x_y", "x_z"]
POS_DES = ["x_des_x", "x_des_y", "x_des_z"]
QUAT = ["quat_x", "quat_y", "quat_z", "quat_w"]
QUAT_DES = ["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate Sysid Validation v1 report.")
    parser.add_argument("--real-csv", required=True)
    parser.add_argument("--baseline-sim-csv", required=True)
    parser.add_argument("--sysid-sim-csv", required=True)
    parser.add_argument("--ablation-json", required=True, help="ablation_outputs.json")
    parser.add_argument("--out-dir", default="/home/tao/Projects/panda_control/data/sysid_v1_report")
    parser.add_argument("--save-plot", action="store_true", default=True)
    return parser.parse_args()


def _load_df(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    req = ["t_s", *POS, *POS_DES, *QUAT, *QUAT_DES]
    miss = [c for c in req if c not in df.columns]
    if miss:
        raise ValueError(f"{path} missing columns: {miss}")
    return df


def _align(real_t: np.ndarray, sim_t: np.ndarray, arr: np.ndarray) -> np.ndarray:
    if len(real_t) == len(sim_t) and np.allclose(real_t, sim_t, atol=1e-4):
        return arr
    out = np.empty((len(real_t), arr.shape[1]), dtype=np.float64)
    for i in range(arr.shape[1]):
        out[:, i] = np.interp(real_t, sim_t, arr[:, i])
    return out


def _sign_align(q: np.ndarray, ref: np.ndarray) -> np.ndarray:
    dot = np.einsum("ij,ij->i", q, ref)
    sign = np.where(dot < 0.0, -1.0, 1.0)
    return q * sign[:, None]


def _theta(q: np.ndarray, ref: np.ndarray) -> np.ndarray:
    dot = np.clip(np.abs(np.einsum("ij,ij->i", q, ref)), 0.0, 1.0)
    return 2.0 * np.arccos(dot)


def metric_real(df_real: pd.DataFrame) -> dict:
    x = df_real[POS].to_numpy()
    x_des = df_real[POS_DES].to_numpy()
    q = _sign_align(df_real[QUAT].to_numpy(), df_real[QUAT_DES].to_numpy())
    q_des = df_real[QUAT_DES].to_numpy()
    e_pos = x_des - x
    th = _theta(q, q_des)
    return {
        "err_pos_rms_xyz_mm": (np.sqrt(np.mean(e_pos**2, axis=0)) * 1000.0).tolist(),
        "err_pos_max_z_mm": float(np.max(np.abs(e_pos[:, 2])) * 1000.0),
        "theta_max_mrad": float(np.max(th) * 1000.0),
    }


def metric_sim(df_real: pd.DataFrame, df_sim: pd.DataFrame) -> dict:
    t = df_real["t_s"].to_numpy()
    x_des = df_real[POS_DES].to_numpy()
    q_des = df_real[QUAT_DES].to_numpy()
    x_sim = _align(t, df_sim["t_s"].to_numpy(), df_sim[POS].to_numpy())
    q_sim = _align(t, df_sim["t_s"].to_numpy(), df_sim[QUAT].to_numpy())
    q_sim = q_sim / np.clip(np.linalg.norm(q_sim, axis=1, keepdims=True), 1e-12, None)
    q_sim = _sign_align(q_sim, q_des)
    e_pos = x_des - x_sim
    th = _theta(q_sim, q_des)
    return {
        "err_pos_rms_xyz_mm": (np.sqrt(np.mean(e_pos**2, axis=0)) * 1000.0).tolist(),
        "err_pos_max_z_mm": float(np.max(np.abs(e_pos[:, 2])) * 1000.0),
        "theta_max_mrad": float(np.max(th) * 1000.0),
    }


def _fmt_xyz(v: list[float]) -> str:
    return f"[{v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f}]"


def _write_markdown(path: Path, content: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def make_plot(out_png: Path, contributions: dict[str, float]):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = list(contributions.keys())
    vals = [contributions[k] for k in labels]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar(labels, vals)
    ax.set_ylabel("Delta z RMS to all_on [mm]")
    ax.set_title("Ablation contribution on held-out trajectory")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    df_real = _load_df(Path(args.real_csv).expanduser().resolve())
    df_base = _load_df(Path(args.baseline_sim_csv).expanduser().resolve())
    df_sysid = _load_df(Path(args.sysid_sim_csv).expanduser().resolve())
    ablation_payload = json.loads(Path(args.ablation_json).expanduser().resolve().read_text(encoding="utf-8"))

    m_real = metric_real(df_real)
    m_base = metric_sim(df_real, df_base)
    m_sysid = metric_sim(df_real, df_sysid)

    ab_metrics = {}
    outputs = ablation_payload.get("outputs", {})
    for name, info in outputs.items():
        sim_csv = Path(info["sim_csv"]).expanduser().resolve()
        if sim_csv.is_file():
            ab_metrics[name] = metric_sim(df_real, _load_df(sim_csv))

    all_on = ab_metrics.get("all_on", m_sysid)
    contrib = {}
    for key, m in ab_metrics.items():
        if key == "all_on":
            continue
        contrib[key] = float(m["err_pos_rms_xyz_mm"][2] - all_on["err_pos_rms_xyz_mm"][2])

    if contrib:
        make_plot(out_dir / "ablation_rms_z.png", contrib)

    z_real = m_real["err_pos_rms_xyz_mm"][2]
    z_sysid = m_sysid["err_pos_rms_xyz_mm"][2]
    rel = abs(z_sysid - z_real) / max(1e-9, z_real)
    pass_generalization = rel < 0.30

    # Heuristic recommendations.
    recs = []
    if contrib:
        small = [k for k, v in contrib.items() if abs(v) < 0.01 * max(1e-9, all_on["err_pos_rms_xyz_mm"][2])]
        if small:
            recs.append(f"ablation影响<1%的组: {', '.join(sorted(small))}，v2可考虑锁死该组参数减少搜索维度。")
    if not pass_generalization:
        recs.append("held-out泛化未达标（z-rms相对误差>=30%），v2建议 step5b+step5c 联合拟合。")
    else:
        recs.append("held-out泛化达标（z-rms相对误差<30%），可进入多轨迹联合拟合验证稳定性。")
    if z_sysid < z_real * 0.9:
        recs.append("sim z-rms 仍偏低，v2建议加入 chirp 或收紧 armature/viscous 搜索上界。")

    lines = []
    lines.append("# SysID Validation v1 Report")
    lines.append("")
    lines.append("## Held-out summary")
    lines.append("")
    lines.append("| metric | real | sim baseline | sim sysid |")
    lines.append("|---|---:|---:|---:|")
    lines.append(
        f"| err_pos_rms_xyz [mm] | `{_fmt_xyz(m_real['err_pos_rms_xyz_mm'])}` | "
        f"`{_fmt_xyz(m_base['err_pos_rms_xyz_mm'])}` | `{_fmt_xyz(m_sysid['err_pos_rms_xyz_mm'])}` |"
    )
    lines.append(
        f"| err_pos_max_z [mm] | {m_real['err_pos_max_z_mm']:.3f} | {m_base['err_pos_max_z_mm']:.3f} | "
        f"{m_sysid['err_pos_max_z_mm']:.3f} |"
    )
    lines.append(
        f"| theta_max [mrad] | {m_real['theta_max_mrad']:.3f} | {m_base['theta_max_mrad']:.3f} | "
        f"{m_sysid['theta_max_mrad']:.3f} |"
    )
    lines.append("")
    lines.append(f"- Held-out success criterion (`|z_sysid-z_real|/z_real < 30%`): **{'PASS' if pass_generalization else 'FAIL'}**")
    lines.append("")
    lines.append("## Ablation contribution (z RMS)")
    lines.append("")
    lines.append("| config | err_pos_rms_z [mm] | delta_to_all_on [mm] |")
    lines.append("|---|---:|---:|")
    for key in sorted(ab_metrics.keys()):
        z = ab_metrics[key]["err_pos_rms_xyz_mm"][2]
        d = z - all_on["err_pos_rms_xyz_mm"][2]
        lines.append(f"| {key} | {z:.3f} | {d:+.3f} |")
    lines.append("")
    lines.append("## V2 recommendations")
    lines.append("")
    for r in recs:
        lines.append(f"- {r}")
    lines.append("")
    lines.append("## Artifacts")
    lines.append("")
    lines.append(f"- ablation inputs: `{Path(args.ablation_json).expanduser().resolve()}`")
    lines.append(f"- bar chart: `{out_dir / 'ablation_rms_z.png'}`")
    lines.append("")

    summary_md = out_dir / "summary.md"
    _write_markdown(summary_md, "\n".join(lines))
    print(f"[report_sysid_v1] wrote {summary_md}")
    if (out_dir / "ablation_rms_z.png").exists():
        print(f"[report_sysid_v1] wrote {out_dir / 'ablation_rms_z.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
