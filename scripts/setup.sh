#!/usr/bin/env bash
# One-time setup for the Jetson Orin NX.
#
# Detects or installs a compatible ROS2 distro, clones GR00T-WholeBodyControl,
# and creates a Python venv with WBC dependencies.
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

# --- Detect or install ROS2 ---
find_ros2_setup() {
    # Return the first ROS2 setup.bash found (prefer newer distros)
    for distro in jazzy humble galactic foxy; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            echo "/opt/ros/$distro/setup.bash"
            return 0
        fi
    done
    return 1
}

ROS2_SETUP=""
if ROS2_SETUP="$(find_ros2_setup)"; then
    DISTRO_NAME="$(basename "$(dirname "$ROS2_SETUP")")"
    echo "[1/3] ROS2 already installed: $DISTRO_NAME ($ROS2_SETUP)"
else
    UBUNTU_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
    echo "[1/3] No ROS2 found. Detected Ubuntu $UBUNTU_CODENAME."

    # Pick the right ROS2 distro for this Ubuntu version
    case "$UBUNTU_CODENAME" in
        jammy)  ROS2_DISTRO="humble" ;;
        focal)  ROS2_DISTRO="foxy" ;;
        noble)  ROS2_DISTRO="jazzy" ;;
        *)
            echo "ERROR: No pre-built ROS2 packages for Ubuntu $UBUNTU_CODENAME."
            echo "Install ROS2 manually, then re-run this script."
            exit 1
            ;;
    esac

    echo "Installing ROS2 $ROS2_DISTRO for Ubuntu $UBUNTU_CODENAME..."
    sudo apt-get update && sudo apt-get install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y "ros-${ROS2_DISTRO}-ros-base" python3-colcon-common-extensions

    ROS2_SETUP="/opt/ros/$ROS2_DISTRO/setup.bash"

    if ! grep -q "source $ROS2_SETUP" ~/.bashrc 2>/dev/null; then
        echo "source $ROS2_SETUP" >> ~/.bashrc
    fi
    echo "ROS2 $ROS2_DISTRO installed."
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

    # Use system Python (matches ROS2), not conda.
    SYS_PYTHON="/usr/bin/python3"
    if [ ! -x "$SYS_PYTHON" ]; then
        SYS_PYTHON="$(which python3)"
        echo "WARNING: /usr/bin/python3 not found, using $SYS_PYTHON"
    fi
    echo "Using Python: $SYS_PYTHON ($($SYS_PYTHON --version 2>&1))"

    "$SYS_PYTHON" -m venv "$WBC_VENV"
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
