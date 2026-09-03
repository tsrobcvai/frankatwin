#!/usr/bin/env bash
# Deploy the frankatwin sysid extension into an official IsaacLab checkout.
#
# Copies three things into <IsaacLab>:
#   1. source/isaaclab_tasks/isaaclab_tasks/direct/franka_sysid/  (gym tasks:
#      Isaac-FrankaTwin-Sysid-v0, Isaac-FrankaTwin-Replay-v0; auto-registered
#      by isaaclab_tasks' package scanner)
#   2. source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd (robot asset)
#   3. scripts/tools/{sysid_franka_osc,apply_sysid_params,replay_python_csv_sim}.py
#
# Usage: ./install_into_isaaclab.sh /path/to/IsaacLab
set -euo pipefail

ISAACLAB="${1:?usage: ./install_into_isaaclab.sh /path/to/IsaacLab}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f "$ISAACLAB/source/isaaclab_tasks/isaaclab_tasks/__init__.py" ]; then
    echo "error: '$ISAACLAB' does not look like an IsaacLab checkout" >&2
    echo "       (missing source/isaaclab_tasks/isaaclab_tasks/__init__.py)" >&2
    exit 1
fi

cp -rv "$HERE/source/isaaclab_tasks/isaaclab_tasks/direct/franka_sysid" \
       "$ISAACLAB/source/isaaclab_tasks/isaaclab_tasks/direct/"

mkdir -p "$ISAACLAB/source/isaaclab_assets/data/Robots/Franka"
cp -v "$HERE/source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd" \
      "$ISAACLAB/source/isaaclab_assets/data/Robots/Franka/"

mkdir -p "$ISAACLAB/scripts/tools"
cp -v "$HERE/scripts/tools/"*.py "$ISAACLAB/scripts/tools/"

echo ""
echo "Installed. Remaining one-time step (inside your IsaacLab python env):"
echo "    pip install cmaes"
