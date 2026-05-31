#!/usr/bin/env python3
"""
act_policyv2.py — ACT inference node for ArmPi Ultra.

Runs a trained ACT policy at 12 Hz on the real robot.
Inference runs in a background thread and fills an action buffer.
The 12 Hz control timer drains the buffer one action per tick.

Architecture (must match training):
    backbone:        resnet18
    hidden_dim:      256
    dim_feedforward: 1024
    enc_layers:      4
    dec_layers:      5
    nheads:          8
    chunk_size:      50   (~4.2 s at 12 Hz)

Setup
-----
  cp scripts/selfPlanning/armpi_constants.py ~/act/
  cp scripts/selfPlanning/armpi_utils.py     ~/act/
  source ~/ros2arm_ws/install/setup.bash

Usage
-----
  # Terminal 1 — camera + crosshair
  ros2 launch armpi_ultra_description teleop_rviz.launch.py

  # Terminal 2 — inference
  python3 scripts/act_policyv2.py
"""

import sys
import os
import math
import time
import threading

import numpy as np

# ── ACT repo ──────────────────────────────────────────────────────────────────
# detr/main.py calls argparse.parse_args() on import.
# Swap argv to a minimal valid set, import, then restore.
ACT_PATH = os.path.expanduser('~/act')
sys.path.insert(0, ACT_PATH)
sys.path.insert(0, os.path.join(ACT_PATH, 'detr'))

_saved_argv = sys.argv[:]
sys.argv    = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
               '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
import torch
torch.set_num_threads(4)
from policy import ACTPolicy          # noqa: E402
sys.argv = _saved_argv

# ── ArmPi helpers ─────────────────────────────────────────────────────────────
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPTS, 'selfPlanning'))
from armpi_constants import NORM_LO, NORM_HI       # noqa: E402
from armpi_utils     import normalize_qpos, denormalize_action  # noqa: E402

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node      import Node
from sensor_msgs.msg import Image

# ── Arm SDK ───────────────────────────────────────────────────────────────────
_SDK = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
        "/Source Code/ROS2/src/driver/ros_robot_controller"
        "/ros_robot_controller")
sys.path.insert(0, _SDK)
from ros_robot_controller_sdk import Board         # noqa: E402

# ── IK ────────────────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/andres/ros2arm_ws/install/kinematics/lib/python3/dist-packages")
from kinematics.ik        import solve, D_BASE, L1, L2, L_TCP   # noqa: E402
from kinematics.transform import (_map, joint1_map, joint2_map,  # noqa: E402
                                   joint3_map, joint4_map, joint5_map)

# ═════════════════════════════════════════════════════════════════════════════
# Configuration
# ═════════════════════════════════════════════════════════════════════════════

# Must match the flags used during training exactly.
POLICY_CONFIG = {
    'lr':              1e-5,
    'num_queries':     50,          # chunk_size — ~4.2 s at 12 Hz
    'kl_weight':       10,
    'hidden_dim':      256,
    'dim_feedforward': 1024,
    'lr_backbone':     1e-5,
    'backbone':        'resnet18',
    'enc_layers':      4,
    'dec_layers':      5,
    'nheads':          8,
    'camera_names':    ['top'],
    'state_dim':       12,   # IK state (6) + joint angles (6) — updated for 12D re-record
}
CHUNK_SIZE = POLICY_CONFIG['num_queries']

CKPT_DIR            = os.path.expanduser('~/ros2arm_ws/checkpoints/pick_place')
SERIAL_PORT         = '/dev/ttyUSB0'
BAUD_RATE           = 1_000_000
CMD_HZ              = 12
CMD_INTERVAL        = 1.0 / CMD_HZ   # ~83 ms
MOVE_DURATION       = 0.08           # servo travel time per command (s)
HOME1_PULSES        = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}
HOME1_MOVE_DURATION = 4.0
MAX_TIMESTEPS       = 700            # safety cutoff


# ═════════════════════════════════════════════════════════════════════════════
# Helpers
# ═════════════════════════════════════════════════════════════════════════════

def pulses_to_ik_state(p: dict) -> np.ndarray:
    """Convert HOME1_PULSES → 6-D IK state [theta, pitch, radius, up_down, gripper_frac, tilt].

    Mirrors the FK used in teleop_ik_v2.py so the initial state is consistent
    with what the policy saw during recording.
    gripper_frac and tilt are 0.0 at home.
    """
    theta = math.radians(_map(p[6], joint1_map))
    j2    = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3    =  math.radians(_map(p[4], joint3_map))
    j4    = -math.radians(_map(p[3], joint4_map)) - math.pi / 2

    q2_std = j2 + math.pi / 2
    q3_std = -j3
    pitch  = j4 + math.pi / 2 - j3 + j2

    elbow_r = L1 * math.cos(q2_std)
    elbow_z = D_BASE + L1 * math.sin(q2_std)
    fa      = q2_std + q3_std
    wrist_r = elbow_r + L2 * math.cos(fa)
    wrist_z = elbow_z + L2 * math.sin(fa)
    tcp_r   = wrist_r + L_TCP * math.cos(pitch)
    tcp_z   = wrist_z + L_TCP * math.sin(pitch)

    z_rel   = tcp_z - D_BASE
    radius  =  tcp_r * math.cos(pitch) + z_rel * math.sin(pitch)
    up_down = -tcp_r * math.sin(pitch) + z_rel * math.cos(pitch)

    return np.array([theta, pitch, radius, up_down, 0.0, 0.0], dtype=np.float32)


def decode_rgb(msg) -> np.ndarray:
    """ROS Image (bgr8 encoding) → (H, W, 3) uint8 RGB."""
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return arr[:, :, ::-1].copy()


def preprocess_image(rgb: np.ndarray) -> torch.Tensor:
    """(H, W, 3) uint8 RGB → (1, 1, 3, H, W) float32 [0, 1].

    Shape: (batch=1, num_cams=1, C, H, W).
    ACTPolicy.__call__ applies ImageNet normalisation internally before ResNet.
    """
    t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0  # (3, H, W)
    return t.unsqueeze(0).unsqueeze(0)                           # (1, 1, 3, H, W)


def pulses_to_joint_angles(p: dict) -> np.ndarray:
    """Servo pulses → 6D joint angles [j1, j2, j3, j4, wrist, gripper_rad]."""
    j1      =  math.radians(_map(p[6], joint1_map))
    j2      = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3      =  math.radians(_map(p[4], joint3_map))
    j4      = -math.radians(_map(p[3], joint4_map)) - math.pi / 2
    wrist   =  math.radians(_map(p[2], joint5_map))
    gripper = (p[1] - 200) / (680 - 200) * 0.785
    return np.array([j1, j2, j3, j4, wrist, gripper], dtype=np.float32)


def read_servo_pulses(board) -> dict:
    """Read actual pulse positions from all 6 servos. Returns {} on failure."""
    pulses = {}
    try:
        for sid in (6, 5, 4, 3, 2, 1):
            result = board.bus_servo_read_position(sid)
            if result is None:
                return {}
            pulses[sid] = result[-1]
    except Exception:
        return {}
    return pulses


# ═════════════════════════════════════════════════════════════════════════════
# Node
# ═════════════════════════════════════════════════════════════════════════════

class ACTPolicyNode(Node):

    def __init__(self):
        super().__init__('act_policyv2')

        # ── Load policy ───────────────────────────────────────────────────────
        self.get_logger().info('Loading policy …')
        sys.argv = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
                    '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
        self.policy = ACTPolicy(POLICY_CONFIG)
        sys.argv = _saved_argv
        self.policy.eval()

        ckpt_path = os.path.join(CKPT_DIR, 'policy_best.ckpt')
        self.policy.load_state_dict(torch.load(ckpt_path, map_location='cpu'))
        self.get_logger().info(f'Loaded: {ckpt_path}')

        # ── Arm ───────────────────────────────────────────────────────────────
        self.get_logger().info(f'Connecting to arm on {SERIAL_PORT} …')
        self.board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
        self.board.enable_reception(True)
        time.sleep(0.3)
        for sid in range(2, 7):
            self.board.bus_servo_enable_torque(sid, False)
            time.sleep(0.02)
        self.get_logger().info('Arm connected.')

        # ── Home ──────────────────────────────────────────────────────────────
        self.get_logger().info('Moving to home1 …')
        self.board.bus_servo_set_position(
            HOME1_MOVE_DURATION,
            [[sid, p] for sid, p in HOME1_PULSES.items()])
        time.sleep(HOME1_MOVE_DURATION + 0.5)

        # IK state: accumulated absolute position, updated each tick.
        self._ik_state     = pulses_to_ik_state(HOME1_PULSES)
        self._last_sent    = self._ik_state.copy()   # reverts to this on IK failure
        # Joint angles: servo readback, updated each control tick.
        self._joint_angles = pulses_to_joint_angles(HOME1_PULSES)
        self.get_logger().info('At home1.')

        # ── Shared state ──────────────────────────────────────────────────────
        self._t          = 0             # tick counter
        self._latest_rgb = None          # most recent camera message
        self._buf        = []            # list of (12,) normalised-delta arrays

        self._obs_lock   = threading.Lock()
        self._state_lock = threading.Lock()
        self._buf_lock   = threading.Lock()
        self._board_lock = threading.Lock()

        # ── Inference thread ──────────────────────────────────────────────────
        # Runs as fast as possible; always overwrites the buffer with a fresh chunk.
        self._stop_evt  = threading.Event()
        self._infer_thr = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thr.start()

        # ── 12 Hz control timer ───────────────────────────────────────────────
        self.create_subscription(Image, '/aurora/rgb/crosshair', self._rgb_cb, 10)
        self._timer = self.create_timer(CMD_INTERVAL, self._control_cb)
        self.get_logger().info('Ready.')

    # ── Camera callback ───────────────────────────────────────────────────────

    def _rgb_cb(self, msg):
        with self._obs_lock:
            self._latest_rgb = msg

    # ── Inference thread ──────────────────────────────────────────────────────

    def _infer_loop(self):
        """Continuously produce action chunks and load them into the buffer."""
        while not self._stop_evt.is_set():

            # Wait for first image
            with self._obs_lock:
                rgb_msg = self._latest_rgb
            if rgb_msg is None:
                time.sleep(0.05)
                continue

            # Snapshot state at inference start
            with self._state_lock:
                ik_snap = self._ik_state.copy()
                ja_snap = self._joint_angles.copy()
                t_start = self._t

            # Build 12D qpos: IK state (6) + joint angles (6)
            qpos_12d = np.concatenate([ik_snap, ja_snap])                # (12,)
            image_t  = preprocess_image(decode_rgb(rgb_msg))             # (1,1,3,H,W)
            qpos_t   = torch.from_numpy(
                normalize_qpos(qpos_12d)).float().unsqueeze(0)           # (1, 12)

            # Run policy
            t0 = time.monotonic()
            with torch.no_grad():
                chunk_t = self.policy(qpos_t, image_t)                   # (1, CHUNK_SIZE, 12)
            elapsed_ms = (time.monotonic() - t0) * 1000.0

            chunk = chunk_t.squeeze(0).numpy()                           # (CHUNK_SIZE, 12)

            # Skip actions that elapsed during inference
            with self._state_lock:
                offset = min(self._t - t_start, CHUNK_SIZE - 1)

            remaining = CHUNK_SIZE - offset
            self.get_logger().info(
                f'infer {elapsed_ms:.0f} ms  offset={offset}  buf→{remaining}')

            # Replace buffer with the full remaining chunk
            with self._buf_lock:
                self._buf = list(chunk[offset:])

    # ── Control loop (12 Hz) ──────────────────────────────────────────────────

    def _control_cb(self):
        if self._t >= MAX_TIMESTEPS:
            self.get_logger().info('MAX_TIMESTEPS reached — stopping.')
            self._stop_evt.set()
            self.destroy_timer(self._timer)
            return

        # Servo readback — update joint angles for next inference observation
        readback = read_servo_pulses(self.board)
        if readback:
            with self._state_lock:
                self._joint_angles = pulses_to_joint_angles(readback)

        # Pop next action; hold current position if buffer is empty
        with self._buf_lock:
            if not self._buf:
                return
            action_norm = np.asarray(self._buf.pop(0), dtype=np.float32)

        # Denormalise — model outputs 12D; only first 6 are IK deltas
        action_raw = denormalize_action(action_norm)[:6]
        with self._state_lock:
            self._ik_state = np.clip(self._ik_state + action_raw, NORM_LO[:6], NORM_HI[:6])
            ik_snap        = self._ik_state.copy()
            self._t       += 1

        theta, pitch, radius, up_down, gripper_frac, tilt = ik_snap

        # IK solve
        result = solve(
            float(theta), float(pitch), float(radius), float(up_down),
            gripper_tilt=float(tilt), gripper=float(gripper_frac))

        if not result.reachable:
            self.get_logger().warn(
                f't={self._t}  IK unreachable — reverting '
                f'(th={theta:.3f} p={pitch:.3f} r={radius:.3f} ud={up_down:.3f})')
            with self._state_lock:
                self._ik_state[:] = self._last_sent
            with self._buf_lock:
                self._buf.clear()   # stale chunk was computed from wrong state
            return

        # Send to arm
        p = result.pulses
        with self._board_lock:
            self.board.bus_servo_set_position(
                MOVE_DURATION,
                [[6, p[6]], [5, p[5]], [4, p[4]],
                 [3, p[3]], [2, p[2]], [1, p[1]]])

        with self._state_lock:
            self._last_sent[:] = ik_snap


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = ACTPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_evt.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
