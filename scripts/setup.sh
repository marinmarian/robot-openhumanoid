#!/usr/bin/env bash
# One-time setup for the Jetson Orin NX.
#
# Installs ROS2 Humble, clones GR00T-WholeBodyControl, creates a Python venv.
# Run once after flashing the Jetson.
#
# Usage:
#   bash scripts/setup.sh

set -euo pipefail

WBC_DIR="${WBC_DIR:-$HOME/GR00T-WholeBodyControl}"
WBC_VENV="${WBC_VENV:-$HOME/wbc_venv}"

echo "=== OpenHumanoid Jetson Setup ==="
echo "WBC:   $WBC_DIR"
echo "Venv:  $WBC_VENV"
echo ""

# --- ROS2 Humble ---
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "[1/3] ROS2 Humble already installed."
else
    echo "[1/3] Installing ROS2 Humble..."
    sudo apt-get update && sudo apt-get install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# --- GR00T-WholeBodyControl ---
if [ -d "$WBC_DIR" ]; then
    echo "[2/3] GR00T-WholeBodyControl already present."
else
    echo "[2/3] Cloning GR00T-WholeBodyControl..."
    sudo apt-get install -y git-lfs
    git lfs install
    git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git "$WBC_DIR"
fi

# --- Python venv ---
if [ -d "$WBC_VENV" ]; then
    echo "[3/3] Virtualenv already exists."
else
    echo "[3/3] Creating virtualenv and installing WBC dependencies..."
    python3 -m venv "$WBC_VENV"
    source "$WBC_VENV/bin/activate"

    pip install --upgrade pip
    pip install numpy scipy mujoco tyro

    if [ -f "$WBC_DIR/decoupled_wbc/requirements.txt" ]; then
        pip install -r "$WBC_DIR/decoupled_wbc/requirements.txt"
    elif [ -f "$WBC_DIR/requirements.txt" ]; then
        pip install -r "$WBC_DIR/requirements.txt"
    else
        echo "WARNING: No requirements.txt found. Install WBC deps manually."
    fi
fi

echo ""
echo "=== Setup complete ==="
echo ""
echo "Launch:       ./scripts/start.sh"
echo "From laptop:  BRIDGE_URL=http://192.168.123.164:8765 uv run python -m realtime.main"
