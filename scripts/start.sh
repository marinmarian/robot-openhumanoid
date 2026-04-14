#!/usr/bin/env bash
# Launch the bridge + WBC control loop on the Jetson Orin NX.
#
# Usage:
#   ./scripts/start.sh              # real robot (default)
#   ./scripts/start.sh sim          # MuJoCo simulation
#   BRIDGE_WITH_HANDS=1 ./scripts/start.sh
#
# Prerequisites — run setup.sh once first.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BRIDGE_PORT="${BRIDGE_PORT:-8765}"
BRIDGE_WITH_HANDS="${BRIDGE_WITH_HANDS:-0}"
INTERFACE="${1:-real}"
WBC_DIR="${WBC_DIR:-$HOME/GR00T-WholeBodyControl}"
CONDA_ENV="${CONDA_ENV:-wbc}"

if [ ! -d "$WBC_DIR" ]; then
    echo "ERROR: GR00T-WholeBodyControl not found at $WBC_DIR"
    echo "Run ./scripts/setup.sh first."
    exit 1
fi

pkill -9 -f run_with_bridge.py 2>/dev/null || true
sleep 0.5

LOOP_ARGS=""
if [ "$INTERFACE" = "real" ]; then
    if [[ "$BRIDGE_WITH_HANDS" == "1" || "$BRIDGE_WITH_HANDS" == "true" || "$BRIDGE_WITH_HANDS" == "yes" ]]; then
        LOOP_ARGS="-- --interface real --with_hands"
        echo "Starting bridge + WBC (real robot, hands enabled)..."
    else
        LOOP_ARGS="-- --interface real --no-with_hands"
        echo "Starting bridge + WBC (real robot, hands disabled; set BRIDGE_WITH_HANDS=1 to enable)..."
    fi
else
    echo "Starting bridge + WBC (simulation)..."
fi

echo "Port:    $BRIDGE_PORT"
echo "WBC:     $WBC_DIR"
echo "Listen:  0.0.0.0:$BRIDGE_PORT"
echo ""

# Activate the conda env (Python 3.10 + ROS2 Humble + WBC deps)
eval "$(conda shell.bash hook)"
conda activate "$CONDA_ENV"

export PYTHONPATH="${WBC_DIR}:${PYTHONPATH:-}"

echo "Python:  $(python3 --version) ($(which python3))"
echo "Conda:   $CONDA_ENV"
echo ""

exec python3 "$PROJECT_DIR/bridge/run_with_bridge.py" --port "$BRIDGE_PORT" $LOOP_ARGS
