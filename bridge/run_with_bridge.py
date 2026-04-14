#!/usr/bin/env python3
from __future__ import annotations

"""
HTTP bridge server + G1 WBC control loop in a single process.

Accepts velocity commands over HTTP and writes them directly to the
WBC neural network policy.  Designed to run natively on the Jetson
Orin NX inside the Unitree G1 robot.

Usage:
    python3 bridge/run_with_bridge.py [--port 8765] [-- <control loop args>]

Example:
    python3 bridge/run_with_bridge.py --port 8765 -- --interface real
"""

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String

KEYBOARD_INPUT_TOPIC = "/keyboard_input"

key_pub = None
_node = None
_wbc_policy = None
arm_controller = None
hand_controller = None
_last_cmd = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
_last_arm_cmd = {"ok": False, "message": "No arm command issued yet."}
_last_hand_cmd = {"ok": False, "message": "No hand command issued yet."}
_last_pick_sequence = {"ok": False, "message": "No pick sequence executed yet."}


class ArmIKController:
    def __init__(self, config):
        from decoupled_wbc.control.main.constants import (
            CONTROL_GOAL_TOPIC,
            DEFAULT_BASE_HEIGHT,
            DEFAULT_NAV_CMD,
        )
        from decoupled_wbc.control.robot_model.instantiation.g1 import instantiate_g1_robot_model
        from decoupled_wbc.control.robot_model.robot_model import ReducedRobotModel
        from decoupled_wbc.control.teleop.solver.body.body_ik_solver import BodyIKSolver
        from decoupled_wbc.control.teleop.solver.body.body_ik_solver_settings import BodyIKSolverSettings
        from decoupled_wbc.control.utils.ros_utils import ROSMsgPublisher

        waist_location = "lower_and_upper_body" if getattr(config, "enable_waist", False) else "lower_body"
        self.full_robot = instantiate_g1_robot_model(
            waist_location=waist_location,
            high_elbow_pose=getattr(config, "high_elbow_pose", False),
        )
        self.upper_body_robot = ReducedRobotModel.from_active_groups(self.full_robot, ["upper_body"])
        self.body_ik_solver = BodyIKSolver(BodyIKSolverSettings())
        self.body_ik_solver.register_robot(self.upper_body_robot)
        self.control_publisher = ROSMsgPublisher(CONTROL_GOAL_TOPIC)

        self.left_frame = self.full_robot.supplemental_info.hand_frame_names["left"]
        self.right_frame = self.full_robot.supplemental_info.hand_frame_names["right"]
        self.upper_body_indices = self.full_robot.get_joint_group_indices("upper_body")
        self.default_base_height = float(DEFAULT_BASE_HEIGHT)
        self.default_nav_cmd = np.asarray(DEFAULT_NAV_CMD, dtype=np.float64)

        q_full = self.upper_body_robot.reduced_to_full_configuration(self.body_ik_solver.configuration.q.copy())
        self.full_robot.cache_forward_kinematics(q_full, auto_clip=False)
        self.default_targets = {
            self.left_frame: self.full_robot.frame_placement(self.left_frame).np.copy(),
            self.right_frame: self.full_robot.frame_placement(self.right_frame).np.copy(),
        }
        self.last_targets = {name: pose.copy() for name, pose in self.default_targets.items()}

    def command_pose(self, payload: dict) -> dict:
        wrist_pose = payload.get("wrist_pose")
        if wrist_pose is None:
            raise ValueError("Missing 'wrist_pose' field")

        flat_pose = np.asarray(wrist_pose, dtype=np.float64).reshape(-1)
        active_arm = str(payload.get("active_arm", "right")).lower()
        body_data, resolved_pose = self._body_data_from_payload(flat_pose, active_arm)

        q_reduced = self.body_ik_solver(body_data)
        q_full = self.upper_body_robot.reduced_to_full_configuration(q_reduced)
        target_upper_body_pose = np.asarray(q_full[self.upper_body_indices], dtype=np.float64)

        t_now = time.monotonic()
        move_time_s = float(payload.get("move_time_s", 1.5))
        target_time = float(payload.get("target_time", t_now + move_time_s))
        base_height_command = float(payload.get("base_height_command", self.default_base_height))
        navigate_cmd = np.asarray(payload.get("navigate_cmd", self.default_nav_cmd), dtype=np.float64)

        goal = {
            "target_upper_body_pose": target_upper_body_pose,
            "wrist_pose": resolved_pose,
            "base_height_command": base_height_command,
            "navigate_cmd": navigate_cmd,
            "target_time": target_time,
            "timestamp": t_now,
        }
        self.control_publisher.publish(goal)
        self.last_targets = {name: pose.copy() for name, pose in body_data.items()}

        return {
            "ok": True,
            "active_arm": active_arm,
            "frame": payload.get("frame", "base_link"),
            "move_time_s": move_time_s,
            "target_time": target_time,
            "target_upper_body_pose": target_upper_body_pose.tolist(),
            "wrist_pose": resolved_pose.tolist(),
            "frame_names": {"left": self.left_frame, "right": self.right_frame},
        }

    def _body_data_from_payload(self, flat_pose: np.ndarray, active_arm: str) -> tuple[dict[str, np.ndarray], np.ndarray]:
        resolved_pose = self._resolve_pose14(flat_pose, active_arm)
        return {
            self.left_frame: self._pose7_to_matrix(resolved_pose[:7]),
            self.right_frame: self._pose7_to_matrix(resolved_pose[7:]),
        }, resolved_pose

    def _resolve_pose14(self, flat_pose: np.ndarray, active_arm: str) -> np.ndarray:
        if flat_pose.size == 14:
            return flat_pose.astype(np.float64)
        if flat_pose.size != 7:
            raise ValueError("'wrist_pose' must contain either 7 values for one arm or 14 values for both arms")

        if active_arm not in {"left", "right"}:
            raise ValueError("'active_arm' must be either 'left' or 'right' when sending a single-arm wrist pose")

        left_pose = self._matrix_to_pose7(self.last_targets[self.left_frame])
        right_pose = self._matrix_to_pose7(self.last_targets[self.right_frame])
        if active_arm == "left":
            left_pose = flat_pose.astype(np.float64)
        else:
            right_pose = flat_pose.astype(np.float64)
        return np.concatenate([left_pose, right_pose])

    def _pose7_to_matrix(self, pose7: np.ndarray) -> np.ndarray:
        if pose7.shape[0] != 7:
            raise ValueError(f"Expected pose vector of length 7, got {pose7.shape[0]}")
        x, y, z, qw, qx, qy, qz = pose7.tolist()
        quat = np.asarray([qx, qy, qz, qw], dtype=np.float64)
        norm = np.linalg.norm(quat)
        if norm == 0.0:
            raise ValueError("Quaternion norm must be non-zero")
        quat /= norm
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = R.from_quat(quat).as_matrix()
        matrix[:3, 3] = [x, y, z]
        return matrix

    def _matrix_to_pose7(self, matrix: np.ndarray) -> np.ndarray:
        quat_xyzw = R.from_matrix(matrix[:3, :3]).as_quat()
        return np.asarray(
            [
                matrix[0, 3],
                matrix[1, 3],
                matrix[2, 3],
                quat_xyzw[3],
                quat_xyzw[0],
                quat_xyzw[1],
                quat_xyzw[2],
            ],
            dtype=np.float64,
        )


class HandController:
    OPEN_TEMPLATE = np.zeros(7, dtype=np.float64)
    RIGHT_CLOSE_TEMPLATE = np.asarray([0.20, -0.40, -1.05, 1.00, 1.18, 1.00, 1.18], dtype=np.float64)
    LEFT_CLOSE_TEMPLATE = np.asarray([-0.20, 0.40, 1.05, -1.00, -1.18, -1.00, -1.18], dtype=np.float64)

    def __init__(self, config):
        if not getattr(config, "with_hands", True):
            raise RuntimeError("Hand control is disabled. Start the bridge/control loop with --with_hands.")

        from decoupled_wbc.control.envs.g1.utils.command_sender import HandCommandSender

        self.senders = {
            "left": HandCommandSender(is_left=True),
            "right": HandCommandSender(is_left=False),
        }
        self.last_commands = {
            "left": self.OPEN_TEMPLATE.copy(),
            "right": self.OPEN_TEMPLATE.copy(),
        }

    def command(self, payload: dict) -> dict:
        active_arm = str(payload.get("active_arm", "right")).lower()
        if active_arm not in {"left", "right"}:
            raise ValueError("'active_arm' must be either 'left' or 'right'.")

        if "hand_q" in payload:
            command = np.asarray(payload["hand_q"], dtype=np.float64).reshape(-1)
            if command.size != 7:
                raise ValueError("'hand_q' must contain exactly 7 joint values.")
            posture = str(payload.get("posture", "custom"))
        else:
            posture = str(payload.get("posture", "open")).lower()
            command = self._posture_command(active_arm, posture, float(payload.get("gripper_width", 0.08)))

        self.senders[active_arm].send_command(command)
        self.last_commands[active_arm] = command.copy()
        return {
            "ok": True,
            "active_arm": active_arm,
            "posture": posture,
            "hand_q": command.tolist(),
            "message": f"Sent {posture} command to the {active_arm} hand.",
        }

    def _posture_command(self, active_arm: str, posture: str, gripper_width: float) -> np.ndarray:
        if posture in {"open", "release"}:
            return self.OPEN_TEMPLATE.copy()
        if posture in {"close", "grasp"}:
            close_template = self.LEFT_CLOSE_TEMPLATE if active_arm == "left" else self.RIGHT_CLOSE_TEMPLATE
            normalized_width = np.clip((gripper_width - 0.02) / 0.10, 0.0, 1.0)
            close_scale = 1.0 - 0.55 * normalized_width
            return close_template * close_scale
        raise ValueError("'posture' must be one of: open, release, close, grasp, custom")


def _sleep_if_needed(duration_s: float) -> None:
    if duration_s > 0.0:
        time.sleep(duration_s)


def _execute_pick_sequence(payload: dict) -> dict:
    global _last_arm_cmd, _last_hand_cmd

    if arm_controller is None:
        return {"ok": False, "error": "Arm IK controller is not ready yet."}

    active_arm = str(payload.get("active_arm", "right")).lower()
    if active_arm not in {"left", "right"}:
        return {"ok": False, "error": "'active_arm' must be either 'left' or 'right'."}

    pregrasp_pose = payload.get("pregrasp_pose")
    grasp_pose = payload.get("grasp_pose")
    retreat_pose = payload.get("retreat_pose")
    if pregrasp_pose is None or grasp_pose is None or retreat_pose is None:
        return {
            "ok": False,
            "error": "'pregrasp_pose', 'grasp_pose', and 'retreat_pose' are all required for /manipulation/pick_sequence.",
        }

    stages = []
    frame = payload.get("frame", "base_link")
    base_height_command = payload.get("base_height_command")
    navigate_cmd = payload.get("navigate_cmd", [0.0, 0.0, 0.0])
    settle_time_s = float(payload.get("settle_time_s", 0.35))
    grip_settle_s = float(payload.get("grip_settle_s", 0.5))
    gripper_width = float(payload.get("gripper_width", 0.08))
    open_hand_first = bool(payload.get("open_hand_first", True))
    close_hand = bool(payload.get("close_hand", True))

    def append_stage(name: str, result: dict, detail: str) -> bool:
        stages.append({"name": name, "ok": bool(result.get("ok")), "detail": detail, "result": result})
        return bool(result.get("ok"))

    def arm_stage(name: str, wrist_pose: list[float], move_time_key: str, default_move_time: float) -> bool:
        nonlocal frame, active_arm, base_height_command, navigate_cmd
        request = {
            "active_arm": active_arm,
            "frame": frame,
            "move_time_s": float(payload.get(move_time_key, default_move_time)),
            "navigate_cmd": navigate_cmd,
            "wrist_pose": wrist_pose,
        }
        if base_height_command is not None:
            request["base_height_command"] = float(base_height_command)
        try:
            result = arm_controller.command_pose(request)
        except Exception as exc:
            result = {"ok": False, "error": f"Arm stage '{name}' failed: {exc}"}
        if result.get("ok"):
            _last_arm_cmd = result
            _sleep_if_needed(float(result.get("move_time_s", 0.0)) + settle_time_s)
        return append_stage(name, result, f"Moved the {active_arm} wrist to the {name.replace('_', ' ')} pose.")

    def hand_stage(name: str, posture: str, wait_s: float) -> bool:
        nonlocal active_arm, gripper_width
        if hand_controller is None:
            result = {"ok": False, "error": "Hand controller is not ready yet. Restart the bridge with hand support enabled."}
            return append_stage(name, result, result["error"])
        try:
            result = hand_controller.command(
                {
                    "active_arm": active_arm,
                    "posture": posture,
                    "gripper_width": gripper_width,
                }
            )
        except Exception as exc:
            result = {"ok": False, "error": f"Hand stage '{name}' failed: {exc}"}
        if result.get("ok"):
            _last_hand_cmd = result
            _sleep_if_needed(wait_s)
        return append_stage(name, result, f"Sent a {posture} command to the {active_arm} hand.")

    if open_hand_first and not hand_stage("open_hand", "open", float(payload.get("hand_open_settle_s", 0.25))):
        return {"ok": False, "active_arm": active_arm, "stages": stages, "error": stages[-1]["result"].get("error", "Failed to open the hand.")}
    if not arm_stage("move_pregrasp", pregrasp_pose, "pregrasp_move_time_s", 1.6):
        return {"ok": False, "active_arm": active_arm, "stages": stages, "error": stages[-1]["result"].get("error", "Failed to reach pregrasp pose.")}
    if not arm_stage("move_grasp", grasp_pose, "grasp_move_time_s", 1.0):
        return {"ok": False, "active_arm": active_arm, "stages": stages, "error": stages[-1]["result"].get("error", "Failed to descend to grasp pose.")}
    if close_hand and not hand_stage("close_hand", "grasp", grip_settle_s):
        return {"ok": False, "active_arm": active_arm, "stages": stages, "error": stages[-1]["result"].get("error", "Failed to close the hand.")}
    if not arm_stage("move_retreat", retreat_pose, "retreat_move_time_s", 1.4):
        return {"ok": False, "active_arm": active_arm, "stages": stages, "error": stages[-1]["result"].get("error", "Failed to retreat after grasp.")}

    return {
        "ok": True,
        "active_arm": active_arm,
        "frame": frame,
        "gripper_width": gripper_width,
        "stages": stages,
        "message": "Executed pregrasp, descend, hand close, and retreat through the bridge.",
    }


def publish_key(key: str):
    msg = String()
    msg.data = key
    key_pub.publish(msg)


def _set_velocity(vx: float, vy: float, vyaw: float):
    """Set locomotion velocity directly on the WBC policy (no quantization)."""
    lb = _wbc_policy.lower_body_policy
    lb.cmd[0] = vx
    lb.cmd[1] = vy
    lb.cmd[2] = vyaw


def _read_body(handler: BaseHTTPRequestHandler) -> dict:
    length = int(handler.headers.get("Content-Length", 0))
    if length == 0:
        return {}
    return json.loads(handler.rfile.read(length))


def _respond(handler: BaseHTTPRequestHandler, code: int, body: dict):
    try:
        payload = json.dumps(body, default=_json_default).encode()
    except Exception as exc:
        payload = json.dumps({"error": f"serialization: {exc}"}).encode()
        code = 500
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json")
    handler.send_header("Content-Length", str(len(payload)))
    handler.end_headers()
    handler.wfile.write(payload)


def _json_default(obj):
    """Make numpy scalars / arrays JSON-serializable."""
    try:
        import numpy as np
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer, np.floating)):
            return obj.item()
    except ImportError:
        pass
    raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")


class BridgeHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        global _last_cmd, _last_arm_cmd, _last_hand_cmd, _last_pick_sequence
        try:
            data = _read_body(self)
        except (json.JSONDecodeError, ValueError) as e:
            _respond(self, 400, {"error": f"Bad JSON: {e}"})
            return

        if self.path == "/move":
            vx = float(data.get("vx", 0.0))
            vy = float(data.get("vy", 0.0))
            vyaw = float(data.get("vyaw", 0.0))
            _set_velocity(vx, vy, vyaw)
            _last_cmd = {"vx": vx, "vy": vy, "vyaw": vyaw}
            print(f"[BRIDGE] MOVE vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f} (direct)")
            _respond(self, 200, {"ok": True, "vx": vx, "vy": vy, "vyaw": vyaw})

        elif self.path == "/stop":
            _set_velocity(0.0, 0.0, 0.0)
            _last_cmd = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
            print("[BRIDGE] STOP (direct)")
            _respond(self, 200, {"ok": True, "stopped": True})

        elif self.path == "/activate":
            publish_key("]")
            print("[BRIDGE] ACTIVATE -> key=']'")
            _respond(self, 200, {"ok": True, "activated": True})

        elif self.path == "/deactivate":
            publish_key("o")
            print("[BRIDGE] DEACTIVATE -> key='o'")
            _respond(self, 200, {"ok": True, "deactivated": True})

        elif self.path == "/key":
            key = data.get("key", "")
            if not key:
                _respond(self, 400, {"error": "Missing 'key' field"})
                return
            publish_key(key)
            print(f"[BRIDGE] KEY '{key}'")
            _respond(self, 200, {"ok": True, "key": key})

        elif self.path == "/arm/pose":
            if arm_controller is None:
                _respond(self, 503, {"ok": False, "error": "Arm IK controller is not ready yet."})
                return
            try:
                result = arm_controller.command_pose(data)
            except ValueError as exc:
                _respond(self, 400, {"ok": False, "error": str(exc)})
                return
            except Exception as exc:
                _respond(self, 500, {"ok": False, "error": f"Arm IK command failed: {exc}"})
                return
            _last_arm_cmd = result
            print(f"[BRIDGE] ARM active_arm={result['active_arm']} target_time={result['target_time']:.3f} frame={result['frame']}")
            _respond(self, 200, result)

        elif self.path == "/hand/command":
            if hand_controller is None:
                _respond(self, 503, {"ok": False, "error": "Hand controller is not ready yet."})
                return
            try:
                result = hand_controller.command(data)
            except ValueError as exc:
                _respond(self, 400, {"ok": False, "error": str(exc)})
                return
            except Exception as exc:
                _respond(self, 500, {"ok": False, "error": f"Hand command failed: {exc}"})
                return
            _last_hand_cmd = result
            print(f"[BRIDGE] HAND active_arm={result['active_arm']} posture={result['posture']}")
            _respond(self, 200, result)

        elif self.path == "/manipulation/pick_sequence":
            result = _execute_pick_sequence(data)
            if result.get("ok"):
                _last_pick_sequence = result
                print(f"[BRIDGE] PICK active_arm={result['active_arm']} stages={len(result['stages'])}")
                _respond(self, 200, result)
            else:
                _last_pick_sequence = result
                _respond(self, 400 if "error" in result else 500, result)

        else:
            _respond(self, 404, {"error": f"Unknown endpoint: {self.path}"})

    def do_GET(self):
        if self.path == "/status":
            actual_cmd = None
            if _wbc_policy is not None:
                try:
                    lb = _wbc_policy.lower_body_policy
                    actual_cmd = list(lb.cmd)
                except Exception:
                    pass
            _respond(
                self,
                200,
                {
                    "ok": True,
                    "last_cmd": _last_cmd,
                    "actual_cmd": actual_cmd,
                    "last_arm_cmd": _last_arm_cmd,
                    "last_hand_cmd": _last_hand_cmd,
                    "last_pick_sequence": _last_pick_sequence,
                    "arm_endpoint_ready": arm_controller is not None,
                    "hand_endpoint_ready": hand_controller is not None,
                    "policy_connected": _wbc_policy is not None,
                },
            )
        else:
            _respond(self, 404, {"error": f"Unknown endpoint: {self.path}"})

    def log_message(self, format, *args):
        # Suppress routine access logs but let Python's default error handling through
        pass

    def log_error(self, format, *args):
        print(f"[BRIDGE] HTTP error: {format % args}", flush=True)


def start_bridge_server(port: int):
    HTTPServer.allow_reuse_address = True
    server = HTTPServer(("0.0.0.0", port), BridgeHandler)
    print(f"[BRIDGE] HTTP server listening on 0.0.0.0:{port}", flush=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def main():
    bridge_args = []
    loop_args = []
    if "--" in sys.argv:
        split_idx = sys.argv.index("--")
        bridge_args = sys.argv[1:split_idx]
        loop_args = sys.argv[split_idx + 1 :]
    else:
        bridge_args = sys.argv[1:]

    parser = argparse.ArgumentParser(description="Bridge + Control Loop launcher")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args(bridge_args)

    if "--keyboard-dispatcher-type" not in loop_args:
        loop_args.extend(["--keyboard-dispatcher-type", "ros"])
        print("[BRIDGE] Auto-adding --keyboard-dispatcher-type ros", flush=True)

    rclpy.init()
    global _node, key_pub, arm_controller, hand_controller
    _node = rclpy.create_node("bridge_server")
    key_pub = _node.create_publisher(String, KEYBOARD_INPUT_TOPIC, 10)
    spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    spin_thread.start()
    print(f"[BRIDGE] ROS2 publisher ready on {KEYBOARD_INPUT_TOPIC}", flush=True)

    time.sleep(0.5)

    sys.argv = ["run_g1_control_loop.py"] + loop_args
    print(f"[BRIDGE] Launching control loop with args: {loop_args}", flush=True)

    # Monkey-patch the WBC policy factory to capture the policy reference.
    # This avoids modifying any GR00T code — we intercept the return value
    # of get_wbc_policy() before the control loop module is even imported.
    global _wbc_policy
    import decoupled_wbc.control.policy.wbc_policy_factory as _policy_factory
    _orig_get_wbc_policy = _policy_factory.get_wbc_policy

    def _intercepted_get_wbc_policy(*args, **kwargs):
        global _wbc_policy
        _wbc_policy = _orig_get_wbc_policy(*args, **kwargs)
        print("[BRIDGE] Captured WBC policy — direct velocity control ready", flush=True)
        return _wbc_policy

    _policy_factory.get_wbc_policy = _intercepted_get_wbc_policy

    import tyro
    from decoupled_wbc.control.main.teleop.configs.configs import ControlLoopConfig
    from decoupled_wbc.control.main.teleop.run_g1_control_loop import main as control_main

    config = tyro.cli(ControlLoopConfig)

    try:
        arm_controller = ArmIKController(config)
        print("[BRIDGE] Arm IK endpoint ready on /arm/pose", flush=True)
    except Exception as exc:
        arm_controller = None
        print(f"[BRIDGE] WARNING: failed to initialize arm IK controller: {exc}", flush=True)

    try:
        hand_controller = HandController(config)
        print("[BRIDGE] Hand endpoint ready on /hand/command", flush=True)
    except Exception as exc:
        hand_controller = None
        print(f"[BRIDGE] WARNING: failed to initialize hand controller: {exc}", flush=True)

    start_bridge_server(args.port)
    control_main(config)


if __name__ == "__main__":
    main()
