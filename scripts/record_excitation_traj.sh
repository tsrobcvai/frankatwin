#!/usr/bin/env bash
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

TARGET_CSV="${TMP_DIR}/${STEM}_target.csv"
TARGET_JSON="${TMP_DIR}/${STEM}_target.json"
REAL_CSV="${DATA_DIR}/${STEM}.csv"
REAL_JSON="${DATA_DIR}/${STEM}.json"

mkdir -p "${TMP_DIR}" "${DATA_DIR}"

echo "[record_excitation] generating excitation target ..."
python "${SCRIPT_DIR}/gen_excitation_traj.py" \
  --out-csv "${TARGET_CSV}" \
  --out-sidecar "${TARGET_JSON}"

echo "[record_excitation] running step5c_excite ..."
"${ROOT_DIR}/build/step5c_excite" "${ROBOT_IP}" \
  --traj-csv "${TARGET_CSV}" \
  --kp-pos 200 \
  --kp-ori 20 \
  --ramp 1.5 \
  --duration 12.0 \
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
if duration < 10.0:
    raise SystemExit(f"duration too short: {duration:.3f}s")
print(f"[record_excitation] rows={len(df)}, duration={duration:.3f}s")
print(f"[record_excitation] abort={meta.get('abort', {})}")
PY

echo "[record_excitation] done."
echo "  target: ${TARGET_CSV}"
echo "  real  : ${REAL_CSV}"
