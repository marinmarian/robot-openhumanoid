#!/usr/bin/env bash
# One-time setup for the Jetson Orin NX.
#
# Creates a conda environment with Python 3.10 (matching the WBC requirement),
# installs ROS2 Humble via robostack, and installs the WBC package.
#
# Usage:
#   bash scripts/setup.sh

set -euo pipefail

WBC_DIR="${WBC_DIR:-$HOME/GR00T-WholeBodyControl}"
CONDA_ENV="${CONDA_ENV:-wbc}"

echo "=== OpenHumanoid Jetson Setup ==="
echo "WBC:        $WBC_DIR"
echo "Conda env:  $CONDA_ENV"
echo ""

# --- Check prerequisites ---
if [ ! -d "$WBC_DIR" ]; then
    echo "[0] Cloning GR00T-WholeBodyControl..."
    sudo apt-get install -y git-lfs 2>/dev/null || true
    git lfs install
    git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git "$WBC_DIR"
else
    echo "[0] GR00T-WholeBodyControl already present."
fi

if ! command -v conda &>/dev/null; then
    echo "ERROR: conda not found. Install Miniconda first:"
    echo "  https://docs.anaconda.com/miniconda/"
    exit 1
fi

# --- Create conda env with Python 3.10 ---
if conda env list | grep -q "^${CONDA_ENV} "; then
    echo "[1/3] Conda env '$CONDA_ENV' already exists."
else
    echo "[1/3] Creating conda env '$CONDA_ENV' with Python 3.10..."
    conda create -n "$CONDA_ENV" python=3.10 -y
fi

echo "[2/3] Installing ROS2 Humble via robostack..."
# Activate inside a subshell-safe way
eval "$(conda shell.bash hook)"
conda activate "$CONDA_ENV"

# robostack provides ROS2 packages compiled for this Python version
conda install -c robostack-staging ros-humble-ros-base -y 2>&1 || {
    echo ""
    echo "If robostack install failed, try:"
    echo "  conda install -c conda-forge -c robostack-staging ros-humble-ros-base"
    exit 1
}

echo "[3/3] Installing WBC package..."
pip install -e "$WBC_DIR/decoupled_wbc/" 2>&1 || {
    echo ""
    echo "WBC install failed. You may need to install PyTorch manually for Jetson:"
    echo "  pip install torch  (or use NVIDIA's Jetson-specific wheel)"
    exit 1
}

echo ""
echo "=== Setup complete ==="
echo ""
echo "Launch:       ./scripts/start.sh"
echo "From laptop:  BRIDGE_URL=http://192.168.123.164:8765 uv run python -m realtime.main"
