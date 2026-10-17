"""Plots, synchronized video, and explicit-frame sim/real comparison."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .data import AXES, load_bundle, write_json
from .math import quaternion_angle, interpolate_quaternions


def pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {"font.size": 10, "axes.grid": True, "grid.alpha": 0.2, "figure.dpi": 120}
    )
    return plt


def metrics(error):
    error = np.asarray(error)
    if len(error) == 0:
        return {"samples": 0}
    return {
        "samples": len(error),
        "bias": error.mean(axis=0).tolist(),
        "rmse": np.sqrt(np.mean(error**2, axis=0)).tolist(),
        "p95_absolute": np.quantile(np.abs(error), 0.95, axis=0).tolist(),
        "max_absolute": np.max(np.abs(error), axis=0).tolist(),
    }


def phase_stats(arrays, key):
    phases = []
    means = []
    stds = []
    counts = []
    for phase in np.unique(arrays["phase"]):
        mask = (arrays["phase"] == phase) & arrays["settled"]
        if mask.sum() < 3:
            continue
        phases.append(int(phase))
        means.append(arrays[key][mask].mean(axis=0))
        stds.append(arrays[key][mask].std(axis=0))
        counts.append(int(mask.sum()))
    return (
        np.asarray(phases),
        np.asarray(means).reshape(-1, 6),
        np.asarray(stds).reshape(-1, 6),
        counts,
    )


def analyze(run, make_video=False, scene=None):
    run = Path(run)
    a, m = load_bundle(run)
    plt = pyplot()
    for frame in ("O", "K"):
        key = f"wrench_{frame}_K"
        truth = f"gravity_{frame}_K"
        fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for i, ax in enumerate(axes.T.flat):
            ax.plot(
                a["t_s"],
                a[key][:, i],
                color="#1774b7",
                lw=1,
                label="Measured (tool compensated)"
                if m["source"] == "sim"
                else "Franka estimate",
            )
            ax.plot(
                a["t_s"],
                a[truth][:, i],
                color="#e28120",
                lw=1.2,
                ls="--",
                label="Gravity only",
            )
            if frame == "O" and "object_dynamics_O_K" in a:
                ax.plot(
                    a["t_s"],
                    a["object_dynamics_O_K"][:, i],
                    color="#3d9d61",
                    alpha=0.65,
                    lw=0.8,
                    label="Object Newton-Euler",
                )
            ax.set_ylabel(AXES[i].upper() + (" [N]" if i < 3 else " [N m]"))
            ax.scatter(
                a["t_s"][a["settled"]][::30],
                a[key][a["settled"], i][::30],
                s=3,
                color="#1774b7",
                alpha=0.35,
            )
        axes[0, 0].legend(fontsize=8)
        axes[-1, 0].set_xlabel("Time [s]")
        axes[-1, 1].set_xlabel("Time [s]")
        fig.suptitle(
            f'{m["source"].upper()} | {m["protocol"]["mass_kg"]:.3f} kg payload | axes {frame}, moment about K\nRobot-on-payload sign; static-hold dots; gravity reference excludes motion inertia'
        )
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(run / f"wrench_{frame}_K.png")
        plt.close(fig)
    phases, mean, std, counts = phase_stats(a, "wrench_K_K")
    _, theory, _, _ = phase_stats(a, "gravity_K_K")
    if len(phases):
        fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for i, ax in enumerate(axes.T.flat):
            ax.errorbar(
                phases,
                mean[:, i],
                yerr=std[:, i],
                fmt="o-",
                capsize=3,
                label="Hold mean +/- temporal std",
            )
            ax.plot(
                phases,
                theory[:, i],
                "x--",
                color="#e28120",
                label="Gravity at measured pose",
            )
            ax.set_ylabel(AXES[i].upper() + (" [N]" if i < 3 else " [N m]"))
        axes[0, 0].legend(fontsize=8)
        axes[-1, 0].set_xlabel("Pose index")
        axes[-1, 1].set_xlabel("Pose index")
        fig.suptitle(
            "Settled poses | K axes, moment about K | error bars are within-run variability"
        )
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        fig.savefig(run / "static_poses.png")
        plt.close(fig)
    report = {
        "source": m["source"],
        "completed": m.get("completed", False),
        "sample_count": len(a["t_s"]),
        "max_sample_interval_s": float(np.diff(a["t_s"]).max()),
        "static_gravity_error_O_K": metrics(
            a["wrench_O_K"][a["settled"]] - a["gravity_O_K"][a["settled"]]
        ),
        "static_gravity_error_K_K": metrics(
            a["wrench_K_K"][a["settled"]] - a["gravity_K_K"][a["settled"]]
        ),
        "pose_indices": phases.tolist(),
        "pose_sample_counts": counts,
        "all_poses_settled": len(phases)
        == len(m["protocol"]["rotation_waypoints_deg"]),
        "unsettled_pose_indices": sorted(
            set(range(len(m["protocol"]["rotation_waypoints_deg"])))
            - set(phases.tolist())
        ),
        "notes": ["This is a sim/theory check, not a measured sim-real gap."]
        if m["source"] == "sim"
        else [],
    }
    if not report["all_poses_settled"]:
        report["notes"].append(
            "Some poses have no settled samples; use their dynamic traces, not a static gravity score. A completed trajectory does not imply every hold settled."
        )
    write_json(run / "analysis.json", report)
    if make_video:
        dashboard_video(run, a, m, scene)
    print("[REPORT] " + str(run / "analysis.json"), flush=True)
    return report


def dashboard_video(run, a, m, scene=None):
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw

    plt = pyplot()
    scene = Path(scene) if scene else Path(run) / "scene.mp4"
    if not scene.exists():
        raise ValueError("scene.mp4 missing; collect sim with --video")
    if scene != Path(run) / "scene.mp4":
        import hashlib

        manifest = json.loads((scene.parent / "render.json").read_text())
        digest = hashlib.sha256((Path(run) / "samples.npz").read_bytes()).hexdigest()
        if manifest.get("measurement_sha256") != digest or not manifest.get(
            "completed"
        ):
            raise ValueError("scene replay does not match this measurement bundle")
    times = a["video_t_s"]
    reader = imageio.get_reader(str(scene))
    fps = reader.get_meta_data()["fps"]
    fig, axes = plt.subplots(3, 2, figsize=(6.4, 7.2), dpi=100, sharex=True)
    fig.subplots_adjust(
        left=0.12, right=0.98, top=0.88, bottom=0.08, hspace=0.27, wspace=0.36
    )
    cursors = []
    for i, ax in enumerate(axes.T.flat):
        ax.plot(a["t_s"], a["wrench_K_K"][:, i], color="#1774b7", lw=0.8)
        ax.plot(a["t_s"], a["gravity_K_K"][:, i], color="#e28120", lw=1, ls="--")
        ax.set_ylabel(AXES[i].upper() + (" [N]" if i < 3 else " [N m]"))
        cursors.append(ax.axvline(0, color="#c43737", lw=1.2, animated=True))
    axes[-1, 0].set_xlabel("Time [s]")
    axes[-1, 1].set_xlabel("Time [s]")
    fig.suptitle(
        "SIM wrist wrench | K axes, about K\nBlue: measured / tool compensated\nOrange: gravity-only reference",
        fontsize=12,
    )
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(fig.bbox)
    writer = imageio.get_writer(
        str(Path(run) / "dashboard.mp4"),
        fps=fps,
        codec="libx264",
        quality=None,
        bitrate="5M",
        macro_block_size=16,
    )
    count = -1
    try:
        for count, frame in enumerate(reader):
            if count >= len(times):
                raise ValueError("video has more frames than recorded timestamps")
            t = float(times[count])
            fig.canvas.restore_region(background)
            for line in cursors:
                line.set_xdata([t, t])
                line.axes.draw_artist(line)
            chart = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
            canvas = Image.new("RGB", (1600, 720), "white")
            canvas.paste(Image.fromarray(frame).resize((960, 720)), (0, 0))
            canvas.paste(Image.fromarray(chart), (960, 0))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle((0, 0, 959, 62), fill=(17, 29, 42))
            draw.text(
                (18, 10),
                f'External force sim-real gap measure | SIM PREVIEW | {m["protocol"]["mass_kg"]:.3f} kg PLA',
                fill="white",
            )
            idx = min(len(a["t_s"]) - 1, np.searchsorted(a["t_s"], t))
            draw.text(
                (18, 33),
                f't = {t:5.2f} s  |  pose {int(a["phase"][idx])}  |  physical grasp  |  no real data yet',
                fill="white",
            )
            writer.append_data(np.asarray(canvas))
            if count % 100 == 0:
                print(f"[VIDEO] {count}/{len(times)} frames", flush=True)
        if count + 1 != len(times):
            raise ValueError("video and timestamps differ in length")
    finally:
        reader.close()
        writer.close()
        plt.close(fig)
    write_json(
        Path(run) / "dashboard.json",
        {
            "scene": str(scene.resolve()),
            "method": "recorded-state visualization"
            if scene != Path(run) / "scene.mp4"
            else "live physics rendering",
            "frames": count + 1,
            "fps": fps,
            "timestamps": "samples.npz/video_t_s",
        },
    )


def compare(sim_run, real_run, out, time_offset=0.0, allow_incomplete=False):
    sa, sm = load_bundle(sim_run)
    ra, rm = load_bundle(real_run)
    if sm["source"] != "sim" or rm["source"] != "real":
        raise ValueError("expected sim and real sources")
    if sm["protocol_sha256"] != rm["protocol_sha256"]:
        raise ValueError("different protocols")
    if sm["empty"] != rm["empty"]:
        raise ValueError("cannot compare loaded and empty runs")
    if not allow_incomplete and (not sm.get("completed") or not rm.get("completed")):
        raise ValueError(
            "incomplete or failed run; --allow-incomplete is for diagnostics only"
        )
    for key in ("EE_T_K", "F_T_EE"):
        if not np.allclose(sm[key], rm[key], atol=1e-6):
            raise ValueError(f"{key} mismatch; replay with --real-run")
    rt = ra["t_s"] + time_offset
    st = sa["t_s"]
    overlap = (rt >= st[0]) & (rt <= st[-1])
    if overlap.sum() < 3:
        raise ValueError("insufficient time overlap; extrapolation is forbidden")
    rt = rt[overlap]

    def interp(key):
        return np.column_stack([np.interp(rt, st, col) for col in sa[key].T])

    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    plt = pyplot()
    report = {
        "sim": str(Path(sim_run).resolve()),
        "real": str(Path(real_run).resolve()),
        "time_offset_applied_to_real_s": time_offset,
        "automatic_time_alignment": False,
        "completed_inputs": bool(sm.get("completed") and rm.get("completed")),
        "overlap_seconds": float(rt[-1] - rt[0]),
        "time_aligned": {},
        "static_pose_comparison": {},
    }
    for frame in ("O", "K"):
        key = f"wrench_{frame}_K"
        sv = interp(key)
        rv = ra[key][overlap]
        error = sv - rv
        report["time_aligned"][key] = metrics(error)
        fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for i, ax in enumerate(axes.T.flat):
            ax.plot(rt, rv[:, i], label="Real", color="#d8781f", lw=1)
            ax.plot(rt, sv[:, i], label="Sim", color="#1774b7", lw=1)
            ax.set_ylabel(AXES[i].upper() + (" [N]" if i < 3 else " [N m]"))
        axes[0, 0].legend()
        axes[-1, 0].set_xlabel("Time [s]")
        axes[-1, 1].set_xlabel("Time [s]")
        fig.suptitle(
            f"Sim-real | axes {frame}, moment about K | no automatic delay fitting"
        )
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.savefig(out / f"gap_{frame}_K.png")
        plt.close(fig)
        fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for i, ax in enumerate(axes.T.flat):
            ax.plot(rt, error[:, i], color="#8b4a99", lw=0.9)
            ax.axhline(0, color="black", lw=0.7, ls="--")
            ax.set_ylabel(AXES[i].upper() + (" [N]" if i < 3 else " [N m]"))
        axes[-1, 0].set_xlabel("Time [s]")
        axes[-1, 1].set_xlabel("Time [s]")
        fig.suptitle(
            f"Sim minus real | axes {frame}, moment about K | no bias subtraction"
        )
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.savefig(out / f"residual_{frame}_K.png")
        plt.close(fig)
        np.savetxt(
            out / f"gap_{frame}_K.csv",
            np.column_stack([rt, sv, rv, error]),
            delimiter=",",
            comments="",
            header=",".join(
                ["t_s"]
                + [
                    f"{tag}_{axis}"
                    for tag in ("sim", "real", "sim_minus_real")
                    for axis in AXES
                ]
            ),
        )
        sp, smean, sstd, sn = phase_stats(sa, key)
        rp, rmean, rstd, rn = phase_stats(ra, key)
        records = []
        for phase in sorted(set(sp) & set(rp)):
            si = int(np.where(sp == phase)[0][0])
            ri = int(np.where(rp == phase)[0][0])
            records.append(
                {
                    "phase": int(phase),
                    "sim_mean": smean[si].tolist(),
                    "real_mean": rmean[ri].tolist(),
                    "gap": (smean[si] - rmean[ri]).tolist(),
                    "sim_std": sstd[si].tolist(),
                    "real_std": rstd[ri].tolist(),
                    "sim_samples": sn[si],
                    "real_samples": rn[ri],
                }
            )
        report["static_pose_comparison"][key] = records
    sq = interpolate_quaternions(st, sa["ee_quat"], rt)
    report["motion_mismatch"] = {
        "position_rmse_m": float(
            np.sqrt(
                np.mean(np.sum((interp("ee_pos") - ra["ee_pos"][overlap]) ** 2, axis=1))
            )
        ),
        "orientation_rmse_rad": float(
            np.sqrt(np.mean(quaternion_angle(sq, ra["ee_quat"][overlap]) ** 2))
        ),
        "joint_rmse_rad": np.sqrt(
            np.mean((interp("q") - ra["q"][overlap]) ** 2, axis=0)
        ).tolist(),
    }
    report["real_zero_estimate_fraction"] = rm.get("zero_estimate_fraction")
    report["notes"] = [
        "Gap includes real model/estimator/filter errors and recorded motion mismatch; it is not a pure sensor calibration error.",
        "Force errors are in N; torque errors are in N m; no mixed-unit six-dimensional norm is reported.",
    ]
    write_json(out / "comparison.json", report)
    print("[COMPARISON] " + str(out / "comparison.json"), flush=True)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    a = sub.add_parser("analyze")
    a.add_argument("run", type=Path)
    a.add_argument("--video", action="store_true")
    a.add_argument(
        "--scene",
        type=Path,
        help="Optional recorded-state render scene.mp4, with render.json provenance.",
    )
    c = sub.add_parser("compare")
    c.add_argument("--sim", type=Path, required=True)
    c.add_argument("--real", type=Path, required=True)
    c.add_argument("--out", type=Path, required=True)
    c.add_argument("--time-offset", type=float, default=0.0)
    c.add_argument("--allow-incomplete", action="store_true")
    args = p.parse_args()
    if args.command == "analyze":
        analyze(args.run, args.video, args.scene)
    else:
        compare(args.sim, args.real, args.out, args.time_offset, args.allow_incomplete)


if __name__ == "__main__":
    main()
