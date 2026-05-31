#!/usr/bin/env python3
"""
act_policyv2.py — ACT inference node for ArmPi Ultra.

Runs a trained ACT policy at 12 Hz on the real robot.

Setup
-----
1. Extract checkpoint zip:
     unzip ~/Downloads/checkpoints_pick_place_2026-05-30.zip \
           -d ~/ros2arm_ws/
   Result: ~/ros2arm_ws/checkpoints/pick_place/policy_best.ckpt
           ~/ros2arm_ws/checkpoints/pick_place/dataset_stats.pkl

2. ACT repo must be cloned at ~/act with patched armpi files:
     cp scripts/selfPlanning/armpi_constants.py ~/act/
     cp scripts/selfPlanning/armpi_utils.py     ~/act/

3. Source workspace before running:
     source ~/ros2arm_ws/install/setup.bash

Usage
-----
  # Terminal 1 — camera + crosshair node
  ros2 launch armpi_ultra_description teleop_rviz.launch.py

  # Terminal 2 — inference
  python3 scripts/act_policyv2.py
"""

import sys
import os
import math
import time
import threading
import pickle

import numpy as np

# ── ACT repo ──────────────────────────────────────────────────────────────────
# detr/main.py calls parse_args() on import; swap argv so it sees valid args,
# then restore ROS argv immediately after the import.
ACT_PATH = os.path.expanduser('~/act')
sys.path.insert(0, ACT_PATH)
sys.path.insert(0, os.path.join(ACT_PATH, 'detr'))
_ros_argv  = sys.argv[:]
sys.argv   = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
              '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
import torch
torch.set_num_threads(4)
from policy import ACTPolicy         # noqa: E402  (ACT repo)
sys.argv = _ros_argv

# ── armpi helpers (canonical copies in selfPlanning/) ─────────────────────────
_SELF = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SELF, 'selfPlanning'))
from armpi_constants import NORM_LO, NORM_HI   # noqa: E402
from armpi_utils import normalize_qpos, denormalize_action  # noqa: E402

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ── Arm SDK ───────────────────────────────────────────────────────────────────
SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board  # noqa: E402

# ── IK ────────────────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/andres/ros2arm_ws/install/kinematics/lib/python3/dist-packages")
from kinematics.ik import solve, D_BASE, L1, L2, L_TCP           # noqa: E402
from kinematics.transform import _map, joint1_map, joint2_map, \
                                  joint3_map, joint4_map          # noqa: E402

# ── Checkpoint ────────────────────────────────────────────────────────────────
CKPT_DIR = os.path.expanduser('~/ros2arm_ws/checkpoints/pick_place')

# ── Policy config — must match training flags exactly ─────────────────────────
POLICY_CONFIG = {
    'lr':              1e-5,
    'num_queries':     100,       # chunk_size (~8.3 s at 12 Hz)
    'kl_weight':       10,
    'hidden_dim':      512,
    'dim_feedforward': 3200,
    'lr_backbone':     1e-5,
    'backbone':        'resnet18',
    'enc_layers':      4,
    'dec_layers':      7,
    'nheads':          8,
    'camera_names':    ['top'],   # RGB only — no depth
    'state_dim':       6,
}
CHUNK_SIZE = POLICY_CONFIG['num_queries']

# Actions to execute per inference cycle before re-inferring with a fresh
# observation. 3 ticks = 0.25 s at 12 Hz — arm moves briefly then holds
# while the next inference runs (~740 ms). Increase for smoother motion.
EXEC_TICKS = 3

# ── Hardware ──────────────────────────────────────────────────────────────────
SERIAL_PORT        = '/dev/ttyUSB0'
BAUD_RATE          = 1_000_000
CMD_HZ             = 12
CMD_INTERVAL       = 1.0 / CMD_HZ   # ~83 ms
MOVE_DURATION      = 0.08           # servo travel time per command (from teleop)

# ── Home position ─────────────────────────────────────────────────────────────
HOME1_PULSES        = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}
HOME1_MOVE_DURATION = 4.0   # seconds

MAX_TIMESTEPS = 700         # safety upper bound on episode length


def pulses_to_ik_state(p: dict) -> np.ndarray:
    """Convert HOME1_PULSES to a 6-D IK state array.

    Returns [theta, pitch, radius, up_down, gripper_frac, tilt] float32.
    Uses the same FK as teleop_ik_v2.py for the first four DOF.
    gripper_frac is 0.0 at home (fully open).
    tilt is 0.0 at home (servo 2 centred at 498 ≈ 500).
    """
    theta  = math.radians(_map(p[6], joint1_map))
    j2     = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3     =  math.radians(_map(p[4], joint3_map))
    j4     = -math.radians(_map(p[3], joint4_map)) - math.pi / 2

    q2_std  = j2 + math.pi / 2
    q3_std  = -j3
    pitch   = j4 + math.pi / 2 - j3 + j2

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
    """ROS Image (bgr8) → (H, W, 3) uint8 RGB."""
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return arr[:, :, ::-1].copy()


def preprocess_rgb(rgb: np.ndarray) -> torch.Tensor:
    """(H, W, 3) uint8 RGB → (1, 1, 3, H, W) float32 in [0, 1].

    Outer dims: (batch=1, num_cameras=1, C, H, W) — matches ACTPolicy forward().
    """
    t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0  # (3, H, W)
    return t.unsqueeze(0).unsqueeze(0)                           # (1, 1, 3, H, W)


# ── Inference node ────────────────────────────────────────────────────────────

class ACTPolicyNode(Node):

    def __init__(self):
        super().__init__('act_policyv2')

        # ── Load policy ───────────────────────────────────────────────────────
        # ACTPolicy.__init__ triggers detr parse_args() — swap argv again.
        self.get_logger().info('Loading policy …')
        sys.argv = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
                    '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
        self.policy = ACTPolicy(POLICY_CONFIG)
        sys.argv = _ros_argv
        self.policy.eval()

        ckpt_path = os.path.join(CKPT_DIR, 'policy_best.ckpt')
        state_dict = torch.load(ckpt_path, map_location='cpu')
        self.policy.load_state_dict(state_dict)
        self.get_logger().info(f'Loaded: {ckpt_path}')

        # dataset_stats.pkl — zeros/ones (no-op normalization, baked into HDF5)
        stats_path = os.path.join(CKPT_DIR, 'dataset_stats.pkl')
        with open(stats_path, 'rb') as f:
            _ = pickle.load(f)   # kept for completeness; normalization uses armpi_utils

        # ── Arm connection ────────────────────────────────────────────────────
        self.get_logger().info(f'Connecting to arm on {SERIAL_PORT} …')
        self.board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
        self.board.enable_reception(True)
        time.sleep(0.3)
        for sid in range(2, 7):                            # engage torque on joints 2–6
            self.board.bus_servo_enable_torque(sid, False) # False = LOAD (SDK quirk)
            time.sleep(0.02)
        self.get_logger().info('Arm connected.')

        # ── Move to home1 and seed IK state ───────────────────────────────────
        self.get_logger().info('Moving to home1 …')
        self.board.bus_servo_set_position(
            HOME1_MOVE_DURATION,
            [[sid, p] for sid, p in HOME1_PULSES.items()])
        time.sleep(HOME1_MOVE_DURATION + 0.5)

        # _smo — the current IK state, applied directly to the servo each tick.
        # During teleop, _smo was the LPF-filtered user input.  At inference we
        # apply policy deltas directly (no additional LPF) because the recorded
        # actions already ARE the deltas of the smoothed trajectory.
        self._smo = pulses_to_ik_state(HOME1_PULSES)
        self.get_logger().info('At home1.')

        self._board_lock = threading.Lock()

        # ── Observations ──────────────────────────────────────────────────────
        self._latest_rgb = None
        self._obs_lock   = threading.Lock()
        self.create_subscription(Image, '/aurora/rgb/crosshair', self._rgb_cb, 10)

        # ── Action buffer ──────────────────────────────────────────────────────
        # Inference thread replaces the whole list; control thread pops from front.
        self._buf      = []          # list of (6,) float32 normalized-delta arrays
        self._buf_lock = threading.Lock()

        # ── Shared state snapshot for inference thread ────────────────────────
        self._t          = 0
        self._state_lock = threading.Lock()
        # Last state successfully sent — used to revert on unreachable IK.
        self._last_sent  = self._smo.copy()

        # ── Inference thread — tight loop, always running ─────────────────────
        self._stop_evt  = threading.Event()
        self._infer_thr = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thr.start()

        # ── 12 Hz control timer ───────────────────────────────────────────────
        self._timer = self.create_timer(CMD_INTERVAL, self._control_cb)
        self.get_logger().info('Ready — running policy at 12 Hz.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _rgb_cb(self, msg):
        with self._obs_lock:
            self._latest_rgb = msg

    # ── Inference loop (background thread) ───────────────────────────────────

    def _infer_loop(self):
        """Runs continuously. Each iteration produces a new action chunk."""
        while not self._stop_evt.is_set():

            # Wait for first image
            with self._obs_lock:
                rgb_msg = self._latest_rgb
            if rgb_msg is None:
                time.sleep(0.05)
                continue

            # Snapshot smoothed state before inference starts.
            # Training data (bag_to_hdf5) recorded the smoothed /teleop/ik_state,
            # so the policy expects smoothed qpos — not the raw accumulated state.
            with self._state_lock:
                qpos_snap = self._smo.copy()
                t_start   = self._t

            # Preprocess
            image_t = preprocess_rgb(decode_rgb(rgb_msg))          # (1,1,3,H,W)
            qpos_t  = torch.from_numpy(
                normalize_qpos(qpos_snap)).float().unsqueeze(0)    # (1, 6)

            # Inference
            t0 = time.monotonic()
            with torch.no_grad():
                raw = self.policy(qpos_t, image_t)                 # (1, CHUNK_SIZE, 6)
            elapsed_ms = (time.monotonic() - t0) * 1000.0

            chunk = raw.squeeze(0).numpy()                         # (CHUNK_SIZE, 6)

            # Skip actions that elapsed during inference
            with self._state_lock:
                ticks_elapsed = self._t - t_start
            offset = min(ticks_elapsed, CHUNK_SIZE - 1)

            self.get_logger().info(
                f'infer {elapsed_ms:.0f} ms  offset={offset}  buf→{CHUNK_SIZE - offset}')

            with self._buf_lock:
                self._buf = list(chunk[offset : offset + EXEC_TICKS])

    # ── Control loop (12 Hz) ──────────────────────────────────────────────────

    def _control_cb(self):
        if self._t >= MAX_TIMESTEPS:
            self.get_logger().info('MAX_TIMESTEPS reached — stopping.')
            self._stop_evt.set()
            self.destroy_timer(self._timer)
            return

        # Pop next action — hold position if buffer is empty
        with self._buf_lock:
            if not self._buf:
                return
            action_norm = np.asarray(self._buf.pop(0), dtype=np.float32)

        # Apply delta directly — no LPF. The stored actions already are the
        # deltas of the teleop's smoothed trajectory, so re-filtering them here
        # would attenuate every move by ALPHA=0.2 and stall the arm.
        action_raw = denormalize_action(action_norm)               # (6,) raw delta
        with self._state_lock:
            self._smo = np.clip(self._smo + action_raw, NORM_LO, NORM_HI)
            smo_snap  = self._smo.copy()
            self._t  += 1

        theta, pitch, radius, up_down, gripper_frac, tilt = smo_snap

        # 3. IK solve
        result = solve(
            float(theta), float(pitch), float(radius), float(up_down),
            gripper_tilt=float(tilt), gripper=float(gripper_frac))

        if not result.reachable:
            # Revert desired and smoothed state to last known good position —
            # mirrors teleop_ik_v2.py behaviour to prevent silent drift.
            self.get_logger().warn(
                f't={self._t}  IK unreachable — reverting state '
                f'(theta={theta:.3f} pitch={pitch:.3f} '
                f'r={radius:.3f} ud={up_down:.3f})')
            with self._state_lock:
                self._smo[:] = self._last_sent
            return

        # 4. Send all 6 servos
        p = result.pulses
        with self._board_lock:
            self.board.bus_servo_set_position(
                MOVE_DURATION,
                [[6, p[6]], [5, p[5]], [4, p[4]],
                 [3, p[3]], [2, p[2]], [1, p[1]]])

        # Track last successfully sent state for revert-on-unreachable
        with self._state_lock:
            self._last_sent[:] = smo_snap


# ── Entry point ───────────────────────────────────────────────────────────────

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
