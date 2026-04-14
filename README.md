# robot-openhumanoid

Onboard bridge for the Unitree G1 robot. Runs the HTTP bridge server and NVIDIA GR00T WBC control loop natively on the Jetson Orin NX — no Docker, no laptop required for motor control.

Part of [OpenHumanoid](https://github.com/alexzh3/OpenHumanoid). This repo contains only the code that runs on the robot. The voice client, capability stack, and OpenClaw agent stay on the laptop (or wherever you run them) and talk to this bridge over HTTP.

## Architecture

```
Laptop (WiFi)                         Robot (Jetson Orin NX)
┌──────────────────┐                  ┌──────────────────────────────────┐
│ Voice Client     │── HTTP ────────→ │ this repo                        │
│ (realtime/ or    │   /move          │                                  │
│  openclaw/)      │   /stop          │ bridge/run_with_bridge.py        │
│                  │   /activate      │ ├─ HTTP server :8765             │
│                  │   /status        │ └─ WBC neural network policy     │
└──────────────────┘                  │       ↓ direct DDS (local)       │
                                      │    Motor control board           │
                                      └──────────────────────────────────┘
```

## Setup (one-time)

SSH into the robot and clone this repo:

```bash
ssh unitree@192.168.123.164    # password: 123
git clone <this-repo-url> ~/robot-openhumanoid
cd ~/robot-openhumanoid
bash scripts/setup.sh
```

This installs ROS2 Humble, clones GR00T-WholeBodyControl, and creates a Python virtualenv with all WBC dependencies.

## Launch

```bash
# On the robot
cd ~/robot-openhumanoid
./scripts/start.sh
```

Verify from your laptop:

```bash
curl http://192.168.123.164:8765/status
```

## Connect from laptop

Point the OpenHumanoid voice client at the robot's bridge:

```bash
# In the main OpenHumanoid repo on your laptop
BRIDGE_URL=http://192.168.123.164:8765 uv run python -m realtime.main
```

Or set `BRIDGE_URL=http://192.168.123.164:8765` in your `.env`.

## HTTP API

| Method | Endpoint                      | Body                                            | Description                          |
| ------ | ----------------------------- | ----------------------------------------------- | ------------------------------------ |
| POST   | `/move`                       | `{"vx": 0.4, "vy": 0.0, "vyaw": 0.0}`         | Set velocity on WBC policy           |
| POST   | `/stop`                       | —                                               | Zero all velocities                  |
| POST   | `/activate`                   | —                                               | Activate walking policy              |
| POST   | `/deactivate`                 | —                                               | Deactivate policy                    |
| POST   | `/key`                        | `{"key": "9"}`                                  | Send raw key event                   |
| POST   | `/arm/pose`                   | `{"active_arm":"right", "wrist_pose":[...7...]}`| Arm IK control                       |
| POST   | `/hand/command`               | `{"active_arm":"right", "posture":"grasp"}`     | Hand open/close                      |
| POST   | `/manipulation/pick_sequence` | `{pregrasp_pose, grasp_pose, retreat_pose, ...}` | Staged pick pipeline                 |
| GET    | `/status`                     | —                                               | Current state                        |

## Configuration

| Variable           | Default                                      | Description                           |
| ------------------ | -------------------------------------------- | ------------------------------------- |
| `BRIDGE_PORT`      | `8765`                                       | HTTP server port                      |
| `BRIDGE_WITH_HANDS`| `0`                                          | Set to `1` to enable hand endpoints   |
| `WBC_DIR`          | `~/GR00T-WholeBodyControl/decoupled_wbc`     | Path to WBC source                    |
| `WBC_VENV`         | `~/wbc_venv`                                 | Python virtualenv for WBC deps        |

## Files

```
robot-openhumanoid/
├── bridge/
│   └── run_with_bridge.py    # HTTP bridge + WBC control loop (single process)
├── scripts/
│   ├── setup.sh              # One-time Jetson setup
│   └── start.sh              # Launch bridge + WBC
├── .gitignore
├── LICENSE
└── README.md
```

## License

MIT
