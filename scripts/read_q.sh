#!/usr/bin/env bash
#
# Convenience wrapper to read the current Franka joint positions once.
#
# Override robot IP via env var:
#   ROBOT_IP=172.16.0.2 ./scripts/read_q.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN="${PROJ_ROOT}/build/read_current_q"

if [[ ! -x "${BIN}" ]]; then
    echo "[read_q] binary not found at ${BIN}. Build first:" >&2
    echo "    cmake -S \"${PROJ_ROOT}\" -B \"${PROJ_ROOT}/build\"" >&2
    echo "    cmake --build \"${PROJ_ROOT}/build\" -j" >&2
    exit 1
fi

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"

# Ensure transitive shared libs (e.g. boost from conda) are resolvable at runtime.
if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/lib" ]]; then
    export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

echo "[read_q] ROBOT_IP=${ROBOT_IP}"
echo "[read_q] Make sure FCI is active and no other process owns the FCI socket."
echo

exec "${BIN}" "${ROBOT_IP}"
