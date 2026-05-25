#!/usr/bin/env bash
#
# One-shot wrapper for held-out excitation data collection (step5c).
#
# Pipeline:
#   1. read_current_pose <ip>    -> tmp/<stem>_anchor.json  (current EE pose)
#   2. gen_excitation_traj.py    -> tmp/<stem>_target.csv   (12 s multi-axis sines)
#   3. step5c_excite <ip> ...    -> data/<stem>.csv         (real-robot log)
#   4. sanity check              -> abort metadata + duration check
#
# The anchor JSON is generated fresh each run from the robot's current pose,
# so the operator does NOT have to manually teach the robot back to the
# step5b anchor before recording. Just put the EE somewhere with ~10 cm of
# free workspace in all 3 axes and run.
#
# Env overrides:
#   DURATION=3.0   -> short safety pre-flight (default 12.0)
#   KP_POS=200     -> kp for step5c (default 200)
#   KP_ORI=20      -> kp_ori for step5c (default 20)
#   RAMP=1.5       -> ramp for step5c (default 1.5)

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <robot_ip> [output_stem]"
  echo "Example: $0 172.16.0.2 step5c_$(date +%Y%m%d_%H%M%S)"
  exit 1
fi

ROBOT_IP="$1"
STEM="${2:-step5c_$(date +%Y%m%d_%H%M%S)}"

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
python "${SCRIPT_DIR}/gen_excitation_traj.py" \
  --base-sidecar "${ANCHOR_JSON}" \
  --out-csv "${TARGET_CSV}" \
  --out-sidecar "${TARGET_JSON}" \
  --duration "${DURATION}"

echo "[record_excitation] running step5c_excite (duration=${DURATION}s) ..."
"${STEP5C_BIN}" "${ROBOT_IP}" \
  --traj-csv "${TARGET_CSV}" \
  --kp-pos "${KP_POS}" \
  --kp-ori "${KP_ORI}" \
  --ramp "${RAMP}" \
  --duration "${DURATION}" \
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
