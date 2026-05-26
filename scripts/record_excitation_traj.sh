#!/usr/bin/env bash
#
# One-shot wrapper for held-out excitation data collection (step5c / step5d).
#
# Pipeline:
#   1. read_current_pose <ip>    -> tmp/<stem>_anchor.json  (current EE pose)
#   2. gen_excitation_traj.py    -> tmp/<stem>_target.csv   (12 s multi-axis sines)
#   3. step5c_excite <ip> ...    -> data/<stem>.csv         (real-robot log)
#   4. sanity check              -> abort metadata + duration check
#
# The anchor JSON is generated fresh each run from the robot's current pose,
# so the operator does NOT have to manually teach the robot back to the
# step5b anchor before recording. Just put the EE somewhere with ~10-15 cm of
# free workspace in all 3 axes (a bit more for step5d, which adds rotation)
# and run.
#
# Defaults are selected per output stem:
#   * STEM starts with "step5c_"  -> step5c amps (4/4/3 cm, no rotation)
#   * STEM starts with "step5d_"  -> step5d amps (10/10/8 cm + 0.25/0.20 rad)
#   * Default STEM is step5d_<timestamp>.
#
# Env overrides (all optional, applied on top of the per-stem defaults):
#   DURATION=3.0          -> short safety pre-flight (default 12.0)
#   KP_POS=200            -> kp for the C++ controller
#   KP_ORI=20             -> kp_ori for the C++ controller
#   RAMP=1.5              -> hold-pose ramp duration [s]
#   AMP_X / AMP_Y / AMP_Z -> low-band position amplitudes [m]
#   AMP_YAW / AMP_ROLL    -> low-band rotation amplitudes [rad]
#                             yaw is about base-z (drives j1);
#                             roll is about EE-z (drives j5/j7).
#   HIGH_BAND_RATIO=0.20  -> high-band amp as a fraction of low-band amp.
#   CART_ABORT            -> runtime |e_pos|_inf abort threshold [m].
#                             step5c default 0.05, step5d default 0.15
#                             (controller has no dx_des feedforward, so
#                             tracking lag scales with amplitude).
#   ORI_ABORT             -> runtime ||e_ori|| abort threshold [rad].
#                             step5c default 0.30, step5d default 0.40.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <robot_ip> [output_stem]"
  echo "Examples:"
  echo "  $0 172.16.0.2                                   # step5d_<ts> (rotation excited)"
  echo "  $0 172.16.0.2 step5c_\$(date +%Y%m%d_%H%M%S)    # legacy step5c (no rotation)"
  exit 1
fi

ROBOT_IP="$1"
STEM="${2:-step5d_$(date +%Y%m%d_%H%M%S)}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TMP_DIR="${ROOT_DIR}/tmp"
DATA_DIR="${ROOT_DIR}/data"

ANCHOR_JSON="${TMP_DIR}/${STEM}_anchor.json"
TARGET_CSV="${TMP_DIR}/${STEM}_target.csv"
TARGET_JSON="${TMP_DIR}/${STEM}_target.json"
REAL_CSV="${DATA_DIR}/${STEM}.csv"
REAL_JSON="${DATA_DIR}/${STEM}.json"

mkdir -p "${TMP_DIR}" "${DATA_DIR}"

DURATION="${DURATION:-12.0}"
KP_POS="${KP_POS:-200}"
KP_ORI="${KP_ORI:-20}"
RAMP="${RAMP:-1.5}"

# Per-stem default amplitudes and matching runtime abort thresholds.
# The abort thresholds need to scale with the commanded amplitude because
# step5c_excite has no dx_des feedforward, so tracking lag scales roughly
# linearly with reference amplitude.  Empirically step5c (4/4/3 cm) peaks
# at ~4.5 cm err_pos_inf and step5d (10/10/8 cm) would peak ~11-12 cm.
if [[ "${STEM}" == step5c_* ]]; then
  AMP_X="${AMP_X:-0.04}"
  AMP_Y="${AMP_Y:-0.04}"
  AMP_Z="${AMP_Z:-0.03}"
  AMP_YAW="${AMP_YAW:-0.0}"
  AMP_ROLL="${AMP_ROLL:-0.0}"
  HIGH_BAND_RATIO="${HIGH_BAND_RATIO:-0.40}"
  CART_ABORT="${CART_ABORT:-0.05}"
  ORI_ABORT="${ORI_ABORT:-0.30}"
else
  # step5d defaults: bigger Cartesian sweep + multi-band base-z and EE-z rotation.
  AMP_X="${AMP_X:-0.10}"
  AMP_Y="${AMP_Y:-0.10}"
  AMP_Z="${AMP_Z:-0.08}"
  AMP_YAW="${AMP_YAW:-0.25}"
  AMP_ROLL="${AMP_ROLL:-0.20}"
  HIGH_BAND_RATIO="${HIGH_BAND_RATIO:-0.20}"
  CART_ABORT="${CART_ABORT:-0.15}"
  ORI_ABORT="${ORI_ABORT:-0.40}"
fi

READ_POSE_BIN="${ROOT_DIR}/build/read_current_pose"
STEP5C_BIN="${ROOT_DIR}/build/step5c_excite"

if [[ ! -x "${READ_POSE_BIN}" ]]; then
  echo "[record_excitation] binary not found: ${READ_POSE_BIN}" >&2
  echo "[record_excitation] build with: cmake --build ${ROOT_DIR}/build --target read_current_pose step5c_excite -j 8" >&2
  exit 2
fi
if [[ ! -x "${STEP5C_BIN}" ]]; then
  echo "[record_excitation] binary not found: ${STEP5C_BIN}" >&2
  exit 2
fi

if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/lib" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

echo "[record_excitation] reading current robot pose ..."
"${READ_POSE_BIN}" "${ROBOT_IP}" \
  --out "${ANCHOR_JSON}" \
  --kp-pos "${KP_POS}" \
  --kp-ori "${KP_ORI}" \
  --ramp "${RAMP}" \
  > /dev/null

echo "[record_excitation] generating excitation target around current anchor ..."
echo "[record_excitation]   stem      = ${STEM}"
echo "[record_excitation]   pos amps  = (${AMP_X}, ${AMP_Y}, ${AMP_Z}) m"
echo "[record_excitation]   ori amps  = yaw=${AMP_YAW} roll=${AMP_ROLL} rad   (high-band ratio ${HIGH_BAND_RATIO})"
python "${SCRIPT_DIR}/gen_excitation_traj.py" \
  --base-sidecar "${ANCHOR_JSON}" \
  --out-csv "${TARGET_CSV}" \
  --out-sidecar "${TARGET_JSON}" \
  --duration "${DURATION}" \
  --amp-x "${AMP_X}" --amp-y "${AMP_Y}" --amp-z "${AMP_Z}" \
  --amp-yaw "${AMP_YAW}" --amp-roll "${AMP_ROLL}" \
  --high-band-ratio "${HIGH_BAND_RATIO}"

echo "[record_excitation] running step5c_excite (duration=${DURATION}s, cart_abort=${CART_ABORT}m, ori_abort=${ORI_ABORT}rad) ..."
"${STEP5C_BIN}" "${ROBOT_IP}" \
  --traj-csv "${TARGET_CSV}" \
  --kp-pos "${KP_POS}" \
  --kp-ori "${KP_ORI}" \
  --ramp "${RAMP}" \
  --duration "${DURATION}" \
  --cart-abort "${CART_ABORT}" \
  --ori-abort "${ORI_ABORT}" \
  --log "${REAL_CSV}" \
  --sidecar "${REAL_JSON}"

echo "[record_excitation] sanity checking output ..."
python - "$REAL_CSV" "$REAL_JSON" <<'PY'
import json
import sys
from pathlib import Path
import pandas as pd

csv_path = Path(sys.argv[1]).resolve()
json_path = Path(sys.argv[2]).resolve()
df = pd.read_csv(csv_path)
meta = json.loads(json_path.read_text())
required = [
    "t_s", "x_des_x", "x_des_y", "x_des_z",
    "x_x", "x_y", "x_z", "quat_x", "quat_y", "quat_z", "quat_w",
]
missing = [c for c in required if c not in df.columns]
if missing:
    raise SystemExit(f"missing columns: {missing}")
if len(df) < 1000:
    raise SystemExit(f"too few rows: {len(df)}")
duration = float(df["t_s"].iloc[-1]) - float(df["t_s"].iloc[0])
if duration < 2.0:
    raise SystemExit(f"duration too short: {duration:.3f}s")
print(f"[record_excitation] rows={len(df)}, duration={duration:.3f}s")
print(f"[record_excitation] abort={meta.get('abort', {})}")
PY

echo "[record_excitation] done."
echo "  anchor: ${ANCHOR_JSON}"
echo "  target: ${TARGET_CSV}"
echo "  real  : ${REAL_CSV}"
echo "  side  : ${REAL_JSON}"
