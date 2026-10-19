#!/usr/bin/env python3
"""Draw a dimensioned payload placement sheet from the acquisition USD and metadata.

Requires numpy, matplotlib and OpenUSD's pxr Python modules. Does not run physics.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
import numpy as np
from pxr import Usd, UsdGeom


def local(prim):
    return np.array(UsdGeom.Xformable(prim).GetLocalTransformation()).T


def joint_frame(prim, side):
    result = np.eye(4)
    quat = prim.GetAttribute(f"physics:localRot{side}").Get()
    w = float(quat.GetReal())
    x, y, z = quat.GetImaginary()
    result[:3, :3] = [
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ]
    result[:3, 3] = prim.GetAttribute(f"physics:localPos{side}").Get()
    return result


def mesh(stage, body, transform):
    prim = stage.GetPrimAtPath(f"/panda/{body}/visuals")
    shape = UsdGeom.Mesh(prim)
    points = np.array(shape.GetPointsAttr().Get())
    matrix = transform @ local(prim)
    points = (points @ matrix[:3, :3].T + matrix[:3, 3]) * 1000
    counts = np.array(shape.GetFaceVertexCountsAttr().Get())
    indices = np.array(shape.GetFaceVertexIndicesAttr().Get())
    polygons, offset = [], 0
    for count in counts:
        polygons.append(points[indices[offset:offset+count]])
        offset += count
    return points, polygons


def silhouette(ax, polygons, axes, color, zorder=2):
    ax.add_collection(PolyCollection(
        [p[:, axes] for p in polygons], facecolors=color, edgecolors="none",
        antialiaseds=False, zorder=zorder,
    ))


def section(ax, polygons, z, color):
    # Intersect mesh faces with a physical cross-section plane, not a projected hull.
    for poly in polygons:
        hits = []
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            if (a[2] <= z < b[2]) or (b[2] <= z < a[2]):
                hits.append(a[:2] + (z-a[2])/(b[2]-a[2])*(b[:2]-a[:2]))
        if len(hits) == 2:
            xy = np.array(hits)
            ax.plot(xy[:, 1], xy[:, 0], color=color, lw=2)


def dimension(ax, a, b, label, horizontal=False, color="#344256", size=10):
    ax.annotate("", xy=a, xytext=b, arrowprops={
        "arrowstyle": "<->", "color": color, "lw": 1.2, "shrinkA": 0, "shrinkB": 0})
    x, y = (np.array(a)+np.array(b))/2
    ax.text(x, y, label, ha="center", va="center", fontsize=size,
            rotation=0 if horizontal else 90, color=color,
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 2})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    meta = json.loads((args.run / "metadata.json").read_text())
    data = np.load(args.run / "samples.npz")
    asset = Path(meta["robot_usd"])
    digest = hashlib.sha256(asset.read_bytes()).hexdigest()
    if digest != meta["robot_usd_sha256"]:
        raise ValueError("USD differs from the recorded acquisition asset")
    stage = Usd.Stage.Open(str(asset))
    tip_joint = stage.GetPrimAtPath("/panda/panda_hand/panda_fingertip_centered_joint")
    hand_to_ee = joint_frame(tip_joint, 0) @ np.linalg.inv(joint_frame(tip_joint, 1))
    ee_from_hand = np.linalg.inv(hand_to_ee)
    # The bundled hand is attached at the flange origin by two fixed joints.
    hand_joint = stage.GetPrimAtPath("/panda/panda_link7/panda_hand_joint")
    sensor_joint = stage.GetPrimAtPath("/panda/force_sensor/force_sensor_joint")
    link7_to_hand = (joint_frame(hand_joint, 0) @ np.linalg.inv(joint_frame(hand_joint, 1))
                     @ joint_frame(sensor_joint, 0) @ np.linalg.inv(joint_frame(sensor_joint, 1)))
    link7_to_flange = np.eye(4)
    link7_to_flange[2, 3] = 0.107
    derived_f_t_ee = np.linalg.inv(link7_to_flange) @ link7_to_hand @ hand_to_ee
    if not np.allclose(derived_f_t_ee, meta["F_T_EE"], atol=2e-6):
        raise ValueError("recorded EE frame differs from the bundled USD fingertip frame")
    dims = np.array(meta["protocol"]["dimensions_m"]) * 1000
    com = np.array(meta["protocol"]["ee_com_m"]) * 1000
    if not np.allclose(dims, [50, 50, 100]) or not np.allclose(com, [0, 0, 25]):
        raise ValueError("this placement sheet is for the 50 x 50 x 100 mm, +25 mm COM protocol")
    hand_points, hand_polygons = mesh(stage, "panda_hand", ee_from_hand)
    fingers = []
    for i, body in enumerate(["panda_leftfinger", "panda_rightfinger"]):
        joint = stage.GetPrimAtPath(f"/panda/panda_hand/panda_finger_joint{i+1}")
        slide = np.eye(4)
        slide[1, 3] = float(data["finger_q"][0, i])
        transform = ee_from_hand @ joint_frame(joint, 0) @ slide @ np.linalg.inv(joint_frame(joint, 1))
        fingers.append(mesh(stage, body, transform))
    tips = [float(p[:, 2].max()) for p, _ in fingers]
    tip_z = float(np.mean(tips))
    top, bottom = com[2]-dims[2]/2, com[2]+dims[2]/2
    flange_z = -float(np.array(meta["F_T_EE"])[2, 3]) * 1000
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                         "svg.fonttype": "none", "pdf.fonttype": 42})
    fig = plt.figure(figsize=(17, 11), facecolor="white")
    dark, grey, orange, teal = "#172b42", "#788694", "#ef8a38", "#087c91"
    fig.text(.045, .955, "PAYLOAD PLACEMENT  /  SIM TO REAL", fontsize=23, weight="bold", color=dark)
    fig.text(.045, .919, "50 x 50 x 100 mm PLA block  |  dimensions in mm  |  geometry from the exact video USD", color="#536577", fontsize=12)
    ax = fig.add_axes([.035, .14, .53, .735])
    ax.set_title("A  Side elevation: both fingers visible (Y-Z plane)", loc="left", fontsize=12, pad=14, weight="bold")
    silhouette(ax, hand_polygons, [1, 2], "#c3ccd4", 1)
    ax.add_patch(Rectangle((-25, top), 50, 100, fc="#fce5ce", ec=orange, lw=2, zorder=2))
    for _, polygons in fingers:
        silhouette(ax, polygons, [1, 2], grey, 3)
    ax.axvline(0, color="#a2afbb", ls=(0,(5,4)), lw=.8, zorder=4)
    ax.plot([-57, 64], [0, 0], color=teal, ls="--", lw=1.3, zorder=5)
    ax.plot([-23, 23], [flange_z, flange_z], color=dark, lw=2)
    ax.text(0, flange_z-7, "F: flange origin / datum plane", ha="center", color=dark, fontsize=10)
    ax.scatter([0], [0], c=teal, s=35, zorder=8)
    ax.annotate("EE origin, z = 0\nreference plane", xy=(0,0), xytext=(-66,15),
                fontsize=9, color=teal, arrowprops={"arrowstyle":"-", "color":teal},
                bbox={"facecolor":"white", "edgecolor":"none", "pad":2}, zorder=9)
    ax.scatter([0], [25], c=dark, s=55, marker="+", lw=2, zorder=7)
    ax.text(4, 27, "COM\n[0, 0, 25]", color=dark, fontsize=10, va="center")
    ax.text(0, 53, "PLA", ha="center", fontsize=20, weight="bold", color="#b25d1c")
    ax.text(-39, -34, "Finger", color=dark, ha="center", fontsize=10)
    ax.text(39, -34, "Finger", color=dark, ha="center", fontsize=10)
    for z in [top, bottom]:
        ax.plot([25, 80], [z,z], color="#b6c0c9", lw=.8)
    dimension(ax, (78,top), (78,bottom), "100.00")
    dimension(ax, (58,top), (58,0), "25.00", color=teal)
    dimension(ax, (58,0), (58,bottom), "75.00", color=teal)
    ax.plot([-25,-25], [bottom,bottom+15], color="#b6c0c9", lw=.8)
    ax.plot([25,25], [bottom,bottom+15], color="#b6c0c9", lw=.8)
    dimension(ax, (-25,bottom+12), (25,bottom+12), "50.00", horizontal=True)
    ax.plot([-25,-85], [bottom,bottom], color="#b6c0c9", lw=.8)
    ax.plot([-23,-85], [flange_z,flange_z], color="#b6c0c9", lw=.8)
    dimension(ax, (-81,flange_z), (-81,bottom), f"{bottom-flange_z:.3f}  flange to far end")
    ax.annotate("+Z EE", xy=(94,65), xytext=(94,35), ha="center", color=teal,
                arrowprops={"arrowstyle":"->", "color":teal, "lw":1.5})
    ax.annotate("+Y EE", xy=(43,100), xytext=(9,100), va="center", color=teal,
                arrowprops={"arrowstyle":"->", "color":teal, "lw":1.5})
    ax.set(xlim=(-108,104), ylim=(108,-131), aspect="equal")
    ax.axis("off")

    cross = fig.add_axes([.60, .585, .35, .275])
    cross.set_title("B  Cross-section at z = -10 mm (inside the grasp)", loc="left", fontsize=11, pad=12, weight="bold")
    cross.add_patch(Rectangle((-25,-25),50,50,fc="#fce5ce",ec=orange,lw=2))
    for _, polygons in fingers:
        section(cross, polygons, -10, grey)
    cross.axhline(0,color="#bcc6ce",ls="--",lw=.8)
    cross.axvline(0,color="#bcc6ce",ls="--",lw=.8)
    cross.scatter([0],[0],c=teal,s=30,zorder=5)
    for y in [-25,25]:
        cross.plot([y,y],[25,36],color="#aebbc6",lw=.8)
    dimension(cross,(-25,34),(25,34),"50.00 (jaw direction)",horizontal=True,size=10)
    dimension(cross,(62,-25),(62,25),"50.00",size=10)
    cross.text(0,-35,"Center in BOTH transverse directions: x = y = 0",ha="center",fontsize=10,color=dark)
    cross.text(-42,0,"Finger",ha="center",color=dark,fontsize=9)
    cross.text(42,0,"Finger",ha="center",color=dark,fontsize=9)
    cross.annotate("+Y",xy=(60,43),xytext=(36,43),va="center",color=teal,arrowprops={"arrowstyle":"->","color":teal})
    cross.annotate("+X",xy=(-61,15),xytext=(-61,-10),ha="center",color=teal,arrowprops={"arrowstyle":"->","color":teal})
    cross.set(xlim=(-69,72),ylim=(-42,48),aspect="equal")
    cross.axis("off")

    fig.text(.60,.535,"C  Physical placement references",fontsize=12,weight="bold",color=dark)
    rows = [
        ("From flange datum F along +Z", "Distance"),
        ("Near end of block", f"{top-flange_z:.3f} mm"),
        ("Block geometric center", f"{com[2]-flange_z:.3f} mm"),
        ("Far end / removable fixture stop", f"{bottom-flange_z:.3f} mm"),
    ]
    for i,(label,value) in enumerate(rows):
        y = .497-i*.032
        fig.text(.60,y,label,fontsize=10.5,color=dark,weight="bold" if i==0 else "normal")
        fig.text(.95,y,value,fontsize=10.5,color=dark,ha="right",weight="bold" if i==0 else "normal")
    fig.text(.60,.352,"Match the installed fingers and frame calibration first.",fontsize=11,weight="bold",color=teal)
    fig.text(.60,.323,
        "1. Align the 100 mm axis with EE +Z; center x and y.\n"
        "2. Seat the far end against a calibrated removable stop.\n"
        "3. Close the jaws; recheck depth and tilt after clamping.\n"
        "4. Remove the stop before recording free motion.",
        fontsize=10.5,color=dark,va="top",linespacing=1.7)
    fig.text(.60,.194,
        f"Modeled distal fingertip: z = +{tip_z:.3f} mm relative to EE.\n"
        f"Near end to that tip: {tip_z-top:.3f} mm; exposed length: {bottom-tip_z:.3f} mm.\n"
        "These are mesh-extremum dimensions, not contact-patch lengths.\n"
        "Verify the same physical datum on real fingers before using them.",
        fontsize=9.5,color="#536577",va="top",linespacing=1.6)
    fig.text(.045,.065,
        "EE = panda_fingertip_centered. Gray profiles and cross-sections come from the recorded robot mesh.\n"
        "Target block pose is dimensioned; finger opening uses the first recorded sample. COM assumes a uniform block.\n"
        "This sheet verifies simulation geometry; the installed real gripper has not been measured.",
        fontsize=9.5,color="#536577",linespacing=1.5)
    args.out.mkdir(parents=True,exist_ok=True)
    for ext in ["png","svg","pdf"]:
        fig.savefig(args.out/f"placement.{ext}",dpi=180,facecolor="white")
    report = {
        "measurement_run":str(args.run.resolve()), "robot_usd_sha256":digest,
        "method":"USD visual mesh projections and plane intersections; no physics rerun",
        "dimensions_mm":dims.tolist(), "target_com_in_ee_mm":com.tolist(),
        "first_recorded_com_in_ee_mm":(data["object_com_in_ee"][0]*1000).tolist(),
        "first_recorded_finger_q_mm":(data["finger_q"][0]*1000).tolist(),
        "derived_F_T_EE":derived_f_t_ee.tolist(),
        "near_end_z_ee_mm":top,"far_end_z_ee_mm":bottom,
        "modeled_distal_tip_z_ee_mm":tips,
        "flange_to_near_end_mm":top-flange_z,
        "flange_to_com_mm":com[2]-flange_z,
        "flange_to_far_end_mm":bottom-flange_z,
        "real_hardware_measured":False,
    }
    (args.out/"placement_geometry.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2),flush=True)
    print(f"[RESULT] {args.out.resolve()}",flush=True)


if __name__ == "__main__":
    main()
