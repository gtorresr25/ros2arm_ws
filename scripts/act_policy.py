#!/usr/bin/env python3
"""
act_policy.py — ACT inference node for ArmPi Ultra.

Loads a trained ACT checkpoint and runs the policy at 12 Hz on the real robot.

Requirements:
  - ACT repo cloned at ~/act, with patched files from act_pi_transfer/patched_files/
    copied in (armpi_constants.py, armpi_utils.py, detr/models/detr_vae.py, detr/main.py)
  - policy_best.ckpt and dataset_stats.pkl in checkpoints/pick_place/
  - source ~/ros2arm_ws/install/setup.bash before running
  - PyTorch, torchvision, einops installed

Usage:
  # Terminal 1 — camera + crosshair
  ros2 launch armpi_ultra_description teleop_rviz.launch.py

  # Terminal 2 — inference
  python3 scripts/act_policy.py
"""

import sys
import os
import math
import time
import threading
import pickle

import numpy as np

# ── ACT repo ──────────────────────────────────────────────────────────────────
# parse_args() in detr/main.py conflicts with ROS2 argv — clear it before import
ACT_PATH = os.path.expanduser('~/act')
sys.path.insert(0, ACT_PATH)
sys.path.insert(0, os.path.join(ACT_PATH, 'detr'))
_ros_argv = sys.argv[:]
sys.argv   = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
              '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
import torch
torch.set_num_threads(4)
from policy import ACTPolicy
sys.argv = _ros_argv

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ── SDK ───────────────────────────────────────────────────────────────────────
SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board

# ── IK ────────────────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/andres/ros2arm_ws/install/kinematics/lib/python3/dist-packages")
from kinematics.ik import solve, D_BASE, L1, L2, L_TCP

# ── Paths ─────────────────────────────────────────────────────────────────────
CKPT_DIR = os.path.expanduser(
    '~/ros2arm_ws/act_pi_transfer/checkpoints/pick_place')         # full model (321 MB)
# CKPT_DIR = os.path.expanduser(
#     '~/ros2arm_ws/act_pi_transfer/checkpoints/pick_place_lite')  # lite model (96 MB)

# ── Hardware ──────────────────────────────────────────────────────────────────
SERIAL_PORT   = '/dev/ttyUSB0'
BAUD_RATE     = 1_000_000
CMD_HZ        = 12
CMD_INTERVAL  = 1.0 / CMD_HZ
MOVE_DURATION = 0.08
ALPHA         = 0.4     # low-pass smoothing (1.0 = no smoothing, matches teleop_ik_v2.py)

# ── Gripper constants (must match teleop_ik_v2.py) ────────────────────────────
GRIP_OPEN     = 200
GRIP_MAX      = 610
GRIP_STEP     = 20
GRIP_STEP_DT  = 0.05
GRIP_STALL_TH = 60

# ── Depth normalisation (must match training) ─────────────────────────────────
DEPTH_MIN_MM = 10.0
DEPTH_MAX_MM = 300.0

# ── Home position ─────────────────────────────────────────────────────────────
HOME1_PULSES      = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}
HOME1_MOVE_DURATION = 4.0   # seconds — same as teleop startup

# Pulse → degree maps (must match teleop_ik_v2.py)
_J1_MAP = [0, 1000, 500, -120,  120,   0]
_J2_MAP = [0, 1000, 500,   30, -210, -90]
_J3_MAP = [0, 1000, 500, -120,  120,   0]
_J4_MAP = [0, 1000, 500,   30, -210, -90]


def _fwd(v, m):
    return ((v - m[2]) / (m[1] - m[0])) * (m[4] - m[3]) + m[5]


def pulses_to_ik_state(p) -> np.ndarray:
    """Convert HOME1_PULSES → (theta, pitch, radius, up_down) float32 array."""
    theta  = math.radians(_fwd(p[6], _J1_MAP))
    j2     = -math.radians(_fwd(p[5], _J2_MAP)) - math.pi / 2
    j3     =  math.radians(_fwd(p[4], _J3_MAP))
    j4     = -math.radians(_fwd(p[3], _J4_MAP)) - math.pi / 2
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
    return np.array([theta, pitch, radius, up_down], dtype=np.float32)

# ── Inference buffering ───────────────────────────────────────────────────────
MAX_TIMESTEPS = 700     # upper bound on episode length

# ── Model config (must match training exactly) ────────────────────────────────
# Full model config (321 MB checkpoint — pick_place):
POLICY_CONFIG = {
    'lr':              1e-5,
    'num_queries':     100,     # chunk_size (~8.3 s at 12 Hz)
    'kl_weight':       10,
    'hidden_dim':      512,
    'dim_feedforward': 3200,
    'lr_backbone':     1e-5,
    'backbone':        'resnet18',
    'enc_layers':      4,
    'dec_layers':      7,
    'nheads':          8,
    'camera_names':    ['top', 'depth'],
    'state_dim':       5,
}

# Lite model config (96 MB checkpoint — pick_place_lite):
# POLICY_CONFIG = {
#     'lr':              1e-5,
#     'num_queries':     30,      # chunk_size (~2.5 s at 12 Hz)
#     'kl_weight':       10,
#     'hidden_dim':      256,
#     'dim_feedforward': 1024,
#     'lr_backbone':     1e-5,
#     'backbone':        'resnet18',
#     'enc_layers':      4,
#     'dec_layers':      7,
#     'nheads':          8,
#     'camera_names':    ['top', 'depth'],
#     'state_dim':       5,
# }

CHUNK_SIZE = POLICY_CONFIG['num_queries']


# ── Gripper helpers (identical logic to teleop_ik_v2.py) ─────────────────────

def grip_until_stall(board, lock, stop_flag, gripper_frac_state):
    with lock:
        board.bus_servo_set_position(0.6, [[1, GRIP_OPEN]])
    gripper_frac_state[0] = 0.0
    time.sleep(0.7)
    pulse = GRIP_OPEN
    while pulse < GRIP_MAX and not stop_flag.is_set():
        pulse = min(pulse + GRIP_STEP, GRIP_MAX)
        gripper_frac_state[0] = (pulse - GRIP_OPEN) / (GRIP_MAX - GRIP_OPEN)
        with lock:
            board.bus_servo_set_position(0.05, [[1, pulse]])
        time.sleep(GRIP_STEP_DT)
        with lock:
            raw = board.bus_servo_read_position(1)
        actual = raw[0] if isinstance(raw, (list, tuple)) and raw else raw
        if actual is not None and (pulse - actual) >= GRIP_STALL_TH:
            gripper_frac_state[0] = max(0.0, min(1.0,
                (actual - GRIP_OPEN) / (GRIP_MAX - GRIP_OPEN)))
            break


def grip_release(board, lock, gripper_frac_state):
    with lock:
        board.bus_servo_set_position(0.5, [[1, GRIP_OPEN]])
    gripper_frac_state[0] = 0.0


# ── Observation preprocessing ─────────────────────────────────────────────────

def decode_rgb(msg) -> np.ndarray:
    """BGR8 image message → (H, W, 3) uint8 RGB."""
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return arr[:, :, ::-1].copy()


def decode_depth(msg) -> np.ndarray:
    """16UC1 depth message → (H, W) uint16 mm."""
    return np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width).copy()


def preprocess_image(rgb: np.ndarray, depth: np.ndarray) -> torch.Tensor:
    """Stack RGB and depth into (1, 2, 3, H, W) float32 tensor."""
    # RGB: (H, W, 3) uint8 → (3, H, W) float32 in [0, 1]
    rgb_t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0

    # Depth: (H, W) uint16 → (3, H, W) float32 in [0, 1]
    depth_f = depth.astype(np.float32)
    depth_f = np.clip(depth_f, DEPTH_MIN_MM, DEPTH_MAX_MM)
    depth_f = (depth_f - DEPTH_MIN_MM) / (DEPTH_MAX_MM - DEPTH_MIN_MM)
    depth_t = torch.from_numpy(np.stack([depth_f, depth_f, depth_f])).float()  # (3, H, W)

    # Stack: (2, 3, H, W) → unsqueeze batch → (1, 2, 3, H, W)
    return torch.stack([rgb_t, depth_t], dim=0).unsqueeze(0)


# ── Inference node ────────────────────────────────────────────────────────────

class ACTPolicyNode(Node):

    def __init__(self):
        super().__init__('act_policy')

        # ── Load policy ───────────────────────────────────────────────────────
        # detr/main.py calls parse_args() during ACTPolicy instantiation —
        # swap argv again so it sees the expected arguments, then restore.
        self.get_logger().info('Loading policy …')
        sys.argv = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
                    '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
        self.policy = ACTPolicy(POLICY_CONFIG)
        sys.argv = _ros_argv
        self.policy.eval()

        ckpt_path = os.path.join(CKPT_DIR, 'policy_best.ckpt')
        state_dict = torch.load(ckpt_path, map_location='cpu')
        self.policy.load_state_dict(state_dict)
        self.get_logger().info(f'Loaded checkpoint: {ckpt_path}')

        stats_path = os.path.join(CKPT_DIR, 'dataset_stats.pkl')
        with open(stats_path, 'rb') as f:
            stats = pickle.load(f)
        self.action_mean = torch.from_numpy(stats['action_mean']).float()
        self.action_std  = torch.from_numpy(stats['action_std']).float()
        self.qpos_mean   = torch.from_numpy(stats['qpos_mean']).float()
        self.qpos_std    = torch.from_numpy(stats['qpos_std']).float()
        self.get_logger().info('Loaded dataset stats.')

        # ── Latest observations (written by callbacks, read by timer) ─────────
        self.latest_rgb   = None
        self.latest_depth = None
        self.obs_lock     = threading.Lock()

        self.create_subscription(Image, '/aurora/rgb/crosshair',
                                 self._rgb_cb, 10)
        self.create_subscription(Image, '/aurora/depth/image_raw',
                                 self._depth_cb, 10)

        # ── Arm connection ────────────────────────────────────────────────────
        self.get_logger().info(f'Connecting to arm on {SERIAL_PORT} …')
        self.board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
        self.board.enable_reception(True)
        time.sleep(0.3)
        # Engage torque on arm joints (servos 2–6)
        for sid in range(2, 7):
            self.board.bus_servo_enable_torque(sid, False)   # False = LOAD
            time.sleep(0.02)
        self.get_logger().info('Arm connected.')

        # Move to home1 and derive initial IK state (mirrors teleop startup)
        self.get_logger().info('Moving to home1 …')
        self.board.bus_servo_set_position(
            HOME1_MOVE_DURATION,
            [[sid, p] for sid, p in HOME1_PULSES.items()])
        time.sleep(HOME1_MOVE_DURATION + 0.5)
        self._arm_ik_state = pulses_to_ik_state(HOME1_PULSES)   # (theta, pitch, radius, up_down)
        self._smo_ik_state = self._arm_ik_state.copy()          # low-pass filter state
        self.get_logger().info('At home1.')

        self.board_lock         = threading.Lock()
        self.grip_thread        = None
        self.grip_stop          = threading.Event()
        self.gripper_frac_state = [0.0]
        self.grip_is_closed     = False

        self.t = 0

        # ── Action buffer ─────────────────────────────────────────────────────
        # Inference runs in background; control loop pops actions at 12 Hz.
        self._chunk         = None   # (CHUNK_SIZE, 5) numpy array
        self._chunk_pos     = 0      # next action index in _chunk
        self._pending_chunk = None   # (chunk, offset) handed off by inference thread
        self._pending_lock  = threading.Lock()

        # ── Inference thread ──────────────────────────────────────────────────
        self._infer_trigger = threading.Event()
        self._stop_infer    = threading.Event()
        self._infer_thread  = threading.Thread(
            target=self._inference_loop, daemon=True)
        self._infer_thread.start()
        self._infer_trigger.set()    # kick off first inference

        # ── Control timer ─────────────────────────────────────────────────────
        self._timer = self.create_timer(CMD_INTERVAL, self._control_cb)
        self.get_logger().info('Ready. Running policy at 12 Hz.')

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _rgb_cb(self, msg):
        with self.obs_lock:
            self.latest_rgb = msg

    def _depth_cb(self, msg):
        with self.obs_lock:
            self.latest_depth = msg

    # ── Inference loop (background thread) ───────────────────────────────────

    def _inference_loop(self):
        while not self._stop_infer.is_set():
            self._infer_trigger.wait()
            self._infer_trigger.clear()
            if self._stop_infer.is_set():
                break

            # Snapshot observations at inference-start time
            with self.obs_lock:
                rgb_msg   = self.latest_rgb
                depth_msg = self.latest_depth
            if rgb_msg is None or depth_msg is None:
                time.sleep(0.05)
                self._infer_trigger.set()   # retry
                continue

            qpos_arr = np.append(self._arm_ik_state,
                                 self.gripper_frac_state[0]).astype(np.float32)
            t_start  = self.t

            # Preprocess + inference
            image_tensor = preprocess_image(decode_rgb(rgb_msg),
                                            decode_depth(depth_msg))
            qpos_norm = ((torch.from_numpy(qpos_arr).float()
                          - self.qpos_mean) / self.qpos_std).unsqueeze(0)

            t0 = time.monotonic()
            with torch.no_grad():
                raw_actions = self.policy(qpos_norm, image_tensor)
            chunk = (raw_actions.squeeze(0) * self.action_std
                     + self.action_mean).numpy()   # (CHUNK_SIZE, 5)
            elapsed = time.monotonic() - t0

            # Offset = steps that executed while inference ran (soft-switch point)
            offset = min(self.t - t_start, CHUNK_SIZE - 1)
            self.get_logger().info(
                f'inference {elapsed*1000:.0f}ms  offset={offset}')

            with self._pending_lock:
                self._pending_chunk = (chunk, offset)

            # Queue next inference immediately
            self._infer_trigger.set()

    # ── Control loop (12 Hz) ──────────────────────────────────────────────────

    def _control_cb(self):
        if self.t >= MAX_TIMESTEPS:
            self.get_logger().info('MAX_TIMESTEPS reached — stopping policy.')
            self._stop_infer.set()
            self._infer_trigger.set()
            self.destroy_timer(self._timer)
            return

        # Pick up new chunk if inference thread has one ready
        with self._pending_lock:
            pending = self._pending_chunk
            self._pending_chunk = None

        if pending is not None:
            chunk, offset = pending
            self._chunk     = chunk
            self._chunk_pos = offset

            # Gripper hard-switch: apply new chunk's decision immediately
            gripper_cmd = chunk[offset][4] > 0.5
            if gripper_cmd and not self.grip_is_closed:
                self.grip_stop.clear()
                self.grip_thread = threading.Thread(
                    target=grip_until_stall,
                    args=(self.board, self.board_lock,
                          self.grip_stop, self.gripper_frac_state),
                    daemon=True)
                self.grip_thread.start()
                self.grip_is_closed = True
                self.get_logger().info(f't={self.t}  grip (chunk switch)')
            elif not gripper_cmd and self.grip_is_closed:
                self.grip_stop.set()
                grip_release(self.board, self.board_lock, self.gripper_frac_state)
                self.grip_is_closed = False
                self.get_logger().info(f't={self.t}  release (chunk switch)')

        # Wait for first chunk
        if self._chunk is None or self._chunk_pos >= CHUNK_SIZE:
            return

        action = self._chunk[self._chunk_pos]
        self._chunk_pos += 1

        # ── Execute arm (servos 2–6, soft-switch: sequential from offset) ─────
        self._smo_ik_state = ALPHA * action[:4] + (1 - ALPHA) * self._smo_ik_state
        theta, pitch, radius, up_down = self._smo_ik_state
        result = solve(float(theta), float(pitch),
                       float(radius), float(up_down),
                       gripper=self.gripper_frac_state[0])

        if result.reachable:
            p = result.pulses
            with self.board_lock:
                self.board.bus_servo_set_position(
                    MOVE_DURATION,
                    [[6, p[6]], [5, p[5]], [4, p[4]], [3, p[3]], [2, p[2]]])
            self._arm_ik_state[:] = self._smo_ik_state

        # ── Execute gripper (servo 1, per-tick) ───────────────────────────────
        gripper_cmd = action[4] > 0.5

        if gripper_cmd and not self.grip_is_closed:
            self.grip_stop.clear()
            self.grip_thread = threading.Thread(
                target=grip_until_stall,
                args=(self.board, self.board_lock,
                      self.grip_stop, self.gripper_frac_state),
                daemon=True)
            self.grip_thread.start()
            self.grip_is_closed = True
            self.get_logger().info(f't={self.t}  grip')

        elif not gripper_cmd and self.grip_is_closed:
            self.grip_stop.set()
            grip_release(self.board, self.board_lock, self.gripper_frac_state)
            self.grip_is_closed = False
            self.get_logger().info(f't={self.t}  release')

        self.t += 1


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ACTPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
