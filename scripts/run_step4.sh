#!/usr/bin/env bash
#
# Convenience wrapper to launch step4_joint_traj with safe defaults.
#
# Step 4 control law:
#   tau_cmd(t) = Kp*(q_des(t) - q) - Kd*dq + c(q,dq)
# where q_des(t) is a single-joint sinusoid around q_center.
#
# Override via env vars, for example:
#   KP=50 JOINT=3 AMP=0.1 FREQ=0.25 ./scripts/run_step4.sh
#
# KD defaults to auto (2*sqrt(KP), critical damping). To deliberately
# under-damp and provoke the closed-loop stability limit, lower it, e.g.:
#   KP=2000 KD=5 ./scripts/run_step4.sh
#
# To disable explicit Coriolis for A/B testing:
#   NO_CORIOLIS=1 ./scripts/run_step4.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN="${PROJ_ROOT}/build/step4_joint_traj"

if [[ ! -x "${BIN}" ]]; then
    echo "[run_step4] binary not found at ${BIN}. Build first:" >&2
    echo "    cmake -S \"${PROJ_ROOT}\" -B \"${PROJ_ROOT}/build\"" >&2
    echo "    cmake --build \"${PROJ_ROOT}/build\" -j" >&2
    exit 1
fi

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
Q_CENTER="${Q_CENTER:-0.12 -0.4 0.0 -2.6 0.0 2.2 0.9}"
JOINT="${JOINT:-3}"
AMP="${AMP:-0.10}"
FREQ="${FREQ:-0.25}"
KP="${KP:-10.0}"
KD="${KD:-}"
DURATION="${DURATION:-8.0}"
RAMP="${RAMP:-1.5}"
AMP_RAMP="${AMP_RAMP:-${RAMP}}"
NO_CORIOLIS="${NO_CORIOLIS:-0}"
PRINT_ERR_EVERY="${PRINT_ERR_EVERY:-100}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DEFAULT="${PROJ_ROOT}/data/step4_${TS}.csv"
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

echo "[run_step4] ROBOT_IP=${ROBOT_IP}"
echo "[run_step4] Q_CENTER=${Q_CENTER}"
echo "[run_step4] JOINT=${JOINT} AMP=${AMP} FREQ=${FREQ}"
echo "[run_step4] KP=${KP} KD=${KD:-auto(2*sqrt(KP))} DURATION=${DURATION} RAMP=${RAMP} AMP_RAMP=${AMP_RAMP}"
echo "[run_step4] NO_CORIOLIS=${NO_CORIOLIS}"
echo "[run_step4] PRINT_ERR_EVERY=${PRINT_ERR_EVERY} ticks (0=off)"
echo "[run_step4] LOG=${LOG_PATH}"
echo "[run_step4] First-run safety: KP defaults to 10 and DURATION to 8 s."
echo "[run_step4] Sweep one joint at a time and raise KP gradually."
echo

# Q_CENTER is passed unquoted so its 7 floats expand into separate argv tokens.
# shellcheck disable=SC2086
exec "${BIN}" "${ROBOT_IP}" \
    --q-center ${Q_CENTER} \
    --joint "${JOINT}" \
    --amp "${AMP}" \
    --freq "${FREQ}" \
    --kp "${KP}" \
    --duration "${DURATION}" \
    --ramp "${RAMP}" \
    --amp-ramp "${AMP_RAMP}" \
    --print-err-every "${PRINT_ERR_EVERY}" \
    --log "${LOG_PATH}" \
    "${EXTRA_ARGS[@]}"
