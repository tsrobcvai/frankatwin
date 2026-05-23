#!/usr/bin/env bash
#
# Convenience wrapper to launch step3_joint_pd_coriolis with sensible defaults.
#
# Step 3 control law:
#   tau_cmd = Kp*(q_des - q) - Kd*dq + c(q,dq)
# where c(q,dq) is from franka::Model::coriolis(). Gravity/friction remain
# implicitly compensated by libfranka.
#
# Override anything via env vars, for example:
#   ROBOT_IP=172.16.0.2 KP=10 DURATION=3 ./scripts/run_step3.sh
#
# To disable explicit Coriolis for A/B testing:
#   NO_CORIOLIS=1 ./scripts/run_step3.sh
#
# Q_DES default is set close to your measured pose, with a small visible offset:
#   [0.12, -0.4, 0.0, -2.6, 0.0, 2.2, 0.9]
# IMPORTANT: move the robot close to this pose (or whatever you pass) BEFORE running.
# The binary will refuse to start if |q_init - q_des|_inf > 0.25 rad.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN="${PROJ_ROOT}/build/step3_joint_pd_coriolis"

if [[ ! -x "${BIN}" ]]; then
    echo "[run_step3] binary not found at ${BIN}. Build first:" >&2
    echo "    cmake -S \"${PROJ_ROOT}\" -B \"${PROJ_ROOT}/build\"" >&2
    echo "    cmake --build \"${PROJ_ROOT}/build\" -j" >&2
    exit 1
fi

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
Q_DES="${Q_DES:-0.12 -0.4 0.0 -2.6 0.0 2.2 0.9}"
KP="${KP:-10.0}"
DURATION="${DURATION:-3.0}"
RAMP="${RAMP:-1.5}"
NO_CORIOLIS="${NO_CORIOLIS:-0}"
PRINT_C_EVERY="${PRINT_C_EVERY:-100}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DEFAULT="${PROJ_ROOT}/data/step3_${TS}.csv"
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

echo "[run_step3] ROBOT_IP=${ROBOT_IP}"
echo "[run_step3] Q_DES=${Q_DES}"
echo "[run_step3] KP=${KP} DURATION=${DURATION} RAMP=${RAMP}"
echo "[run_step3] NO_CORIOLIS=${NO_CORIOLIS}"
echo "[run_step3] PRINT_C_EVERY=${PRINT_C_EVERY} ticks (0=off)"
echo "[run_step3] LOG=${LOG_PATH}"
echo "[run_step3] First-run safety: KP defaults to 10 and DURATION to 3 s."
echo "[run_step3] Crank KP up only after a clean 3 s run."
echo

# Q_DES is passed unquoted so its 7 floats expand into separate argv tokens.
# shellcheck disable=SC2086
exec "${BIN}" "${ROBOT_IP}" \
    --q-des ${Q_DES} \
    --kp "${KP}" \
    --duration "${DURATION}" \
    --ramp "${RAMP}" \
    --print-c-every "${PRINT_C_EVERY}" \
    --log "${LOG_PATH}" \
    "${EXTRA_ARGS[@]}"
