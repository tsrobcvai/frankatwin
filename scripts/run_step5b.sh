#!/usr/bin/env bash
#
# Convenience wrapper to launch step5b_cart_pose with conservative defaults.
#
# Step 5b control law (6D Cartesian pose PD, no Lambda):
#   tau_cmd = J^T * [Kp_pos*(x_des-x)-Kd_pos*v ; Kp_ori*e_o-Kd_ori*w] + c(q,dq)
# where position trajectory is optional single-axis sinusoid, and orientation
# target is held at startup anchor.
#
# Override via env vars, for example:
#   KP_POS=200 KP_ORI=20 AMP=0.01 AXIS=z ./scripts/run_step5b.sh
#
# Active orientation tracking (default ORI_AMP_DEG=0 -> hold anchor):
#   ORI_AMP_DEG=5 ORI_AXIS=z ORI_FREQ=0.25 ./scripts/run_step5b.sh
#   ORI_FRAME=ee  -> rotate around tool-frame axis (default base frame)
#
# KD defaults to auto (2*sqrt(KP)).
#
# To disable explicit Coriolis:
#   NO_CORIOLIS=1 ./scripts/run_step5b.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN="${PROJ_ROOT}/build/step5b_cart_pose"

if [[ ! -x "${BIN}" ]]; then
    echo "[run_step5b] binary not found at ${BIN}. Build first:" >&2
    echo "    cmake -S \"${PROJ_ROOT}\" -B \"${PROJ_ROOT}/build\"" >&2
    echo "    cmake --build \"${PROJ_ROOT}/build\" -j" >&2
    exit 1
fi

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
AXIS="${AXIS:-z}"
AMP="${AMP:-0.0}"
FREQ="${FREQ:-0.25}"
ORI_AXIS="${ORI_AXIS:-z}"
ORI_AMP_DEG="${ORI_AMP_DEG:-0.0}"
ORI_FREQ="${ORI_FREQ:-0.25}"
ORI_FRAME="${ORI_FRAME:-base}"
KP_POS="${KP_POS:-100.0}"
KD_POS="${KD_POS:-}"
KP_ORI="${KP_ORI:-20.0}"
KD_ORI="${KD_ORI:-}"
DURATION="${DURATION:-8.0}"
RAMP="${RAMP:-1.5}"
AMP_RAMP="${AMP_RAMP:-${RAMP}}"
NO_CORIOLIS="${NO_CORIOLIS:-0}"
PRINT_ERR_EVERY="${PRINT_ERR_EVERY:-100}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DEFAULT="${PROJ_ROOT}/data/step5b_${TS}.csv"
LOG_PATH="${LOG:-${LOG_DEFAULT}}"

# Sidecar JSON lives next to the CSV: same dir, same stem, .json suffix.
# Override with SIDECAR=path. Pass empty string to disable.
if [[ "${LOG_PATH}" == *.csv ]]; then
    SIDECAR_DEFAULT="${LOG_PATH%.csv}.json"
else
    SIDECAR_DEFAULT="${LOG_PATH}.json"
fi
SIDECAR_PATH="${SIDECAR:-${SIDECAR_DEFAULT}}"

# Ensure transitive shared libs (e.g. boost from conda) are resolvable at runtime.
if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/lib" ]]; then
    export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

mkdir -p "$(dirname "${LOG_PATH}")"
if [[ -n "${SIDECAR_PATH}" ]]; then
    mkdir -p "$(dirname "${SIDECAR_PATH}")"
fi

EXTRA_ARGS=()
if [[ "${NO_CORIOLIS}" == "1" ]]; then
    EXTRA_ARGS+=(--no-coriolis)
fi
if [[ -n "${KD_POS}" ]]; then
    EXTRA_ARGS+=(--kd-pos "${KD_POS}")
fi
if [[ -n "${KD_ORI}" ]]; then
    EXTRA_ARGS+=(--kd-ori "${KD_ORI}")
fi

echo "[run_step5b] ROBOT_IP=${ROBOT_IP}"
echo "[run_step5b] AXIS=${AXIS} AMP=${AMP} FREQ=${FREQ}"
echo "[run_step5b] ORI_AXIS=${ORI_AXIS} ORI_AMP_DEG=${ORI_AMP_DEG} ORI_FREQ=${ORI_FREQ} ORI_FRAME=${ORI_FRAME}"
echo "[run_step5b] KP_POS=${KP_POS} KD_POS=${KD_POS:-auto(2*sqrt(KP_POS))}"
echo "[run_step5b] KP_ORI=${KP_ORI} KD_ORI=${KD_ORI:-auto(2*sqrt(KP_ORI))}"
echo "[run_step5b] DURATION=${DURATION} RAMP=${RAMP} AMP_RAMP=${AMP_RAMP}"
echo "[run_step5b] NO_CORIOLIS=${NO_CORIOLIS}"
echo "[run_step5b] PRINT_ERR_EVERY=${PRINT_ERR_EVERY} ticks (0=off)"
echo "[run_step5b] LOG=${LOG_PATH}"
echo "[run_step5b] SIDECAR=${SIDECAR_PATH:-(disabled)}"
echo "[run_step5b] First-run safety: KP_POS defaults to 100, AMP=0 and ORI_AMP_DEG=0 (hold)."
echo "[run_step5b] Compare stretched vs folded poses with same KP/AMP."
echo

CMD=( "${BIN}" "${ROBOT_IP}"
    --axis "${AXIS}"
    --amp "${AMP}"
    --freq "${FREQ}"
    --ori-axis "${ORI_AXIS}"
    --ori-amp-deg "${ORI_AMP_DEG}"
    --ori-freq "${ORI_FREQ}"
    --ori-frame "${ORI_FRAME}"
    --kp-pos "${KP_POS}"
    --kp-ori "${KP_ORI}"
    --duration "${DURATION}"
    --ramp "${RAMP}"
    --amp-ramp "${AMP_RAMP}"
    --print-err-every "${PRINT_ERR_EVERY}"
    --log "${LOG_PATH}" )

if [[ -n "${SIDECAR_PATH}" ]]; then
    CMD+=( --sidecar "${SIDECAR_PATH}" )
fi

CMD+=( "${EXTRA_ARGS[@]}" )

exec "${CMD[@]}"
