#!/usr/bin/env bash
# Launch the bridge + WBC control loop on the Jetson Orin NX.
#
# Usage:
#   ./scripts/start.sh              # real robot (default)
#   ./scripts/start.sh sim          # MuJoCo simulation
#   BRIDGE_WITH_HANDS=1 ./scripts/start.sh
#
# Prerequisites — run setup.sh once first.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BRIDGE_PORT="${BRIDGE_PORT:-8765}"
BRIDGE_WITH_HANDS="${BRIDGE_WITH_HANDS:-0}"
INTERFACE="${1:-real}"
WBC_DIR="${WBC_DIR:-$HOME/GR00T-WholeBodyControl/decoupled_wbc}"
WBC_VENV="${WBC_VENV:-$HOME/wbc_venv}"

if [ ! -d "$WBC_DIR" ]; then
    echo "ERROR: GR00T-WholeBodyControl not found at $WBC_DIR"
    echo "Run ./scripts/setup.sh first, or set WBC_DIR."
    exit 1
fi

if [ ! -d "$WBC_VENV" ]; then
    echo "ERROR: WBC virtualenv not found at $WBC_VENV"
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

source /opt/ros/humble/setup.bash
source "$WBC_VENV/bin/activate"
export PYTHONPATH="${WBC_DIR}/src:${PYTHONPATH:-}"

exec python3 "$PROJECT_DIR/bridge/run_with_bridge.py" --port "$BRIDGE_PORT" $LOOP_ARGS
