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
WBC_DIR="${WBC_DIR:-$HOME/GR00T-WholeBodyControl}"
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

# Source whichever ROS2 distro is installed (Humble, Foxy, etc.)
ROS2_SETUP=""
for distro in jazzy humble galactic foxy; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        ROS2_SETUP="/opt/ros/$distro/setup.bash"
        break
    fi
done
if [ -z "$ROS2_SETUP" ]; then
    echo "ERROR: No ROS2 installation found in /opt/ros/."
    echo "Run ./scripts/setup.sh first."
    exit 1
fi
echo "ROS2:    $ROS2_SETUP"

# Deactivate conda if active — its Python won't work with ROS2's C extensions.
if [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    echo "Deactivating conda ($CONDA_DEFAULT_ENV) to use system Python..."
    eval "$(conda shell.bash deactivate 2>/dev/null)" || { export PATH="${PATH//$CONDA_PREFIX\/bin:/}"; unset CONDA_DEFAULT_ENV CONDA_PREFIX; }
fi

set +u
source "$ROS2_SETUP"
source "$WBC_VENV/bin/activate"
set -u
export PYTHONPATH="${WBC_DIR}:${PYTHONPATH:-}"

exec python3 "$PROJECT_DIR/bridge/run_with_bridge.py" --port "$BRIDGE_PORT" $LOOP_ARGS
