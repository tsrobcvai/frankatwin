#!/usr/bin/env bash
#
# Convenience wrapper to launch step5_cart_pd with conservative defaults.
#
# Step 5 control law (position-only Cartesian PD, no Lambda):
#   tau_cmd = J_p^T * (Kp*(x_des-x) - Kd*dx) + c(q,dq)
# where x_des defaults to x_anchor (captured at start), with optional
# single-axis sinusoid.
#
# Override via env vars, for example:
#   KP=200 AMP=0.01 AXIS=z ./scripts/run_step5.sh
#
# KD defaults to auto (2*sqrt(KP), critical damping). To under-damp:
#   KP=500 KD=10 AMP=0.01 ./scripts/run_step5.sh
#
# To disable explicit Coriolis:
#   NO_CORIOLIS=1 ./scripts/run_step5.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN="${PROJ_ROOT}/build/step5_cart_pd"

if [[ ! -x "${BIN}" ]]; then
    echo "[run_step5] binary not found at ${BIN}. Build first:" >&2
    echo "    cmake -S \"${PROJ_ROOT}\" -B \"${PROJ_ROOT}/build\"" >&2
    echo "    cmake --build \"${PROJ_ROOT}/build\" -j" >&2
    exit 1
fi

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
AXIS="${AXIS:-z}"
AMP="${AMP:-0.0}"
FREQ="${FREQ:-0.25}"
KP="${KP:-50.0}"
KD="${KD:-}"
DURATION="${DURATION:-5.0}"
RAMP="${RAMP:-1.5}"
AMP_RAMP="${AMP_RAMP:-${RAMP}}"
NO_CORIOLIS="${NO_CORIOLIS:-0}"
PRINT_ERR_EVERY="${PRINT_ERR_EVERY:-100}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DEFAULT="${PROJ_ROOT}/data/step5_${TS}.csv"
LOG_PATH="${LOG:-${LOG_DEFAULT}}"

# Ensure transitive shared libs (e.g. boost from conda) are resolvable at runtime.
if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/lib" ]]; then
    export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

mkdir -p "$(dirname "${LOG_PATH}")"

EXTRA_ARGS=()
if [[ "${NO_CORIOLIS}" == "1" ]]; then
    EXTRA_ARGS+=(--no-coriolis)
fi
if [[ -n "${KD}" ]]; then
    EXTRA_ARGS+=(--kd "${KD}")
fi

echo "[run_step5] ROBOT_IP=${ROBOT_IP}"
echo "[run_step5] AXIS=${AXIS} AMP=${AMP} FREQ=${FREQ}"
echo "[run_step5] KP=${KP} KD=${KD:-auto(2*sqrt(KP))} DURATION=${DURATION} RAMP=${RAMP} AMP_RAMP=${AMP_RAMP}"
echo "[run_step5] NO_CORIOLIS=${NO_CORIOLIS}"
echo "[run_step5] PRINT_ERR_EVERY=${PRINT_ERR_EVERY} ticks (0=off)"
echo "[run_step5] LOG=${LOG_PATH}"
echo "[run_step5] First-run safety: KP defaults to 50 and AMP defaults to 0 (hold)."
echo "[run_step5] Compare stretched vs folded poses with same KP/AMP."
echo

exec "${BIN}" "${ROBOT_IP}" \
    --axis "${AXIS}" \
    --amp "${AMP}" \
    --freq "${FREQ}" \
    --kp "${KP}" \
    --duration "${DURATION}" \
    --ramp "${RAMP}" \
    --amp-ramp "${AMP_RAMP}" \
    --print-err-every "${PRINT_ERR_EVERY}" \
    --log "${LOG_PATH}" \
    "${EXTRA_ARGS[@]}"
