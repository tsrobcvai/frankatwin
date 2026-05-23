#!/usr/bin/env bash
#
# Convenience wrapper to launch step1_joint_pd with sensible defaults.
#
# Override anything via env vars, for example:
#   ROBOT_IP=172.16.0.2 KP=10 DURATION=3 ./scripts/run_step1.sh
#
# Q_DES default is Franka's "ready" pose (libfranka examples convention):
#   [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]
# IMPORTANT: move the robot to this pose (or whatever you pass) BEFORE running.
# The binary will refuse to start if |q_init - q_des|_inf > 0.10 rad.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN="${PROJ_ROOT}/build/step1_joint_pd"

if [[ ! -x "${BIN}" ]]; then
    echo "[run_step1] binary not found at ${BIN}. Build first:" >&2
    echo "    cmake -S \"${PROJ_ROOT}\" -B \"${PROJ_ROOT}/build\"" >&2
    echo "    cmake --build \"${PROJ_ROOT}/build\" -j" >&2
    exit 1
fi

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
Q_DES="${Q_DES:-0 -0.785398 0 -2.356194 0 1.570796 0.785398}"
KP="${KP:-10.0}"
DURATION="${DURATION:-3.0}"
RAMP="${RAMP:-1.5}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DEFAULT="${PROJ_ROOT}/data/step1_${TS}.csv"
LOG_PATH="${LOG:-${LOG_DEFAULT}}"

mkdir -p "$(dirname "${LOG_PATH}")"

echo "[run_step1] ROBOT_IP=${ROBOT_IP}"
echo "[run_step1] Q_DES=${Q_DES}"
echo "[run_step1] KP=${KP} DURATION=${DURATION} RAMP=${RAMP}"
echo "[run_step1] LOG=${LOG_PATH}"
echo "[run_step1] First-run safety: KP defaults to 10 and DURATION to 3 s."
echo "[run_step1] Crank KP up only after a clean 3 s run."
echo

# Q_DES is passed unquoted so its 7 floats expand into separate argv tokens.
# shellcheck disable=SC2086
exec "${BIN}" "${ROBOT_IP}" \
    --q-des ${Q_DES} \
    --kp "${KP}" \
    --duration "${DURATION}" \
    --ramp "${RAMP}" \
    --log "${LOG_PATH}"
