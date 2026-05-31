#!/usr/bin/env python3
"""
dummy_policy.py — Validates the ACT inference pipeline without a trained model.

Runs the identical 12 Hz control loop as act_policyv2.py but replaces the
neural network with a deterministic action so you can verify end-to-end:

  1. Arm connects and homes to home1 correctly
  2. 12 Hz control loop is stable
  3. IK solve + servo commands work
  4. Servo readback → joint angles pipeline produces sensible values
  5. Camera topic is live
  6. IK-unreachable fallback (revert + clear) works

Modes
-----
  --mode hold   (default)
      Zero action every tick — arm stays at home1.
      Prints the full 12D observation every second.

  --mode wiggle
      Exercises each IK DOF in sequence, one at a time:
        pitch → radius → up_down → gripper_frac → tilt
      Each DOF gets 2 full sine cycles (8 s) at ±10° (or equivalent).
      The sine returns to zero at the end of each segment, so the arm
      comes back to home1 before the next DOF starts.

Usage
-----
  source ~/ros2arm_ws/install/setup.bash

  python3 scripts/dummy_policy.py               # hold, 20 s
  python3 scripts/dummy_policy.py --mode wiggle # full sequence ~40 s
  python3 scripts/dummy_policy.py --mode hold --ticks 60

What to check (wiggle mode)
---------------------------
  Each DOF is announced in the log, e.g.:
    [wiggle] DOF 1/6 — theta  (±10.0°, 8 s)

  For each segment watch:
    theta    : IK th column oscillates ±10° (side to side)
    pitch    : IK p column oscillates ±10°
    radius   : IK r column oscillates ±2 cm
    up_down  : IK ud column oscillates ±2 cm
    gripper  : IK gr column 0 → 0.4 → 0  (clipped at 0 on negative half)
    tilt     : IK tilt column oscillates ±10°

  After each segment the arm returns to home1 — verify in RViz.
  JA columns should track the IK columns with a small lag (servo inertia).
  No 'IK unreachable' warnings should appear for these amplitudes.
"""

import sys
import os
import math
import time
import threading
import argparse

import numpy as np

# ── ArmPi helpers ─────────────────────────────────────────────────────────────
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPTS, 'selfPlanning'))
from armpi_constants import NORM_LO, NORM_HI   # (12,)

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node      import Node
from sensor_msgs.msg import Image

# ── Arm SDK ───────────────────────────────────────────────────────────────────
try:
    from ros_robot_controller_sdk import Board
except ImportError:
    _SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
                 "/Source Code/ROS2/src/driver/ros_robot_controller"
                 "/ros_robot_controller")
    sys.path.insert(0, _SDK_PATH)
    from ros_robot_controller_sdk import Board

# ── IK ────────────────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/andres/ros2arm_ws/install/kinematics/lib/python3/dist-packages")
from kinematics.ik        import solve, D_BASE, L1, L2, L_TCP
from kinematics.transform import (_map, joint1_map, joint2_map,
                                   joint3_map, joint4_map, joint5_map)

# ═════════════════════════════════════════════════════════════════════════════
# Configuration
# ═════════════════════════════════════════════════════════════════════════════

SERIAL_PORT         = '/dev/ttyUSB0'
BAUD_RATE           = 1_000_000
CMD_HZ              = 12
CMD_INTERVAL        = 1.0 / CMD_HZ
MOVE_DURATION       = 0.08
HOME1_PULSES        = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}
HOME1_MOVE_DURATION = 4.0

# ── Wiggle sequence ────────────────────────────────────────────────────────────
# Each DOF gets WIGGLE_CYCLES full sine cycles before moving to the next.
# The sine returns to 0 at the end of each complete cycle, so the arm
# comes back to home1 automatically.
WIGGLE_CYCLES  = 2               # full cycles per DOF
WIGGLE_PERIOD  = 48              # ticks per cycle = 4 s at 12 Hz
WIGGLE_TICKS   = WIGGLE_CYCLES * WIGGLE_PERIOD   # 96 ticks = 8 s per DOF

# (label, dof_index_in_ik_state, amplitude, unit_for_display)
#   angular DOFs : ±10° = ±0.175 rad
#   radius       : ±2 cm (physically similar to 10° of arm rotation)
#   up_down      : ±2 cm
#   gripper_frac : 0 → 0.4 (clipped at 0 on negative half — gripper opens then closes)
WIGGLE_SEQUENCE = [
    ('theta',        0, math.radians(10), 'deg'),
    ('pitch',        1, math.radians(10), 'deg'),
    ('radius',       2, 0.020,            'm'  ),
    ('up_down',      3, 0.020,            'm'  ),
    ('gripper_frac', 4, 0.40,             'frac'),
    ('tilt',         5, math.radians(10), 'deg'),
]

TOTAL_WIGGLE_TICKS = len(WIGGLE_SEQUENCE) * WIGGLE_TICKS   # 6 × 96 = 576 = 48 s


# ═════════════════════════════════════════════════════════════════════════════
# Helpers
# ═════════════════════════════════════════════════════════════════════════════

def pulses_to_ik_state(p: dict) -> np.ndarray:
    """Servo pulses → 6D IK state. FK identical to teleop_ik_v2.py."""
    theta = math.radians(_map(p[6], joint1_map))
    j2    = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3    =  math.radians(_map(p[4], joint3_map))
    j4    = -math.radians(_map(p[3], joint4_map)) - math.pi / 2

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


def pulses_to_joint_angles(p: dict) -> np.ndarray:
    """Servo pulses → 6D joint angles [j1,j2,j3,j4,wrist,gripper_rad]."""
    j1      =  math.radians(_map(p[6], joint1_map))
    j2      = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3      =  math.radians(_map(p[4], joint3_map))
    j4      = -math.radians(_map(p[3], joint4_map)) - math.pi / 2
    wrist   =  math.radians(_map(p[2], joint5_map))
    gripper = (p[1] - 200) / (680 - 200) * 0.785
    return np.array([j1, j2, j3, j4, wrist, gripper], dtype=np.float32)


def read_servo_pulses(board) -> dict:
    """Read actual pulses from all 6 servos. Returns {} on failure."""
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
# Wiggle action generator
# ═════════════════════════════════════════════════════════════════════════════

def wiggle_action(t: int) -> tuple[np.ndarray, int, bool]:
    """Return (delta_6d, dof_seq_idx, is_new_dof) for global tick t.

    Uses delta = sin(t) - sin(t-1) so the IK state traces a clean sine wave
    and returns exactly to 0 at the end of each complete cycle set.
    Returns a zero delta and dof_seq_idx=-1 once the sequence is done.
    """
    if t >= TOTAL_WIGGLE_TICKS:
        return np.zeros(6, dtype=np.float32), -1, False

    dof_seq_idx = t // WIGGLE_TICKS   # which DOF we're on
    t_local     = t %  WIGGLE_TICKS   # tick within this DOF's segment
    is_new_dof  = (t_local == 0)

    _, dof_idx, amp, _ = WIGGLE_SEQUENCE[dof_seq_idx]

    # Target position traces A * sin(2π * t_local / WIGGLE_PERIOD)
    # Delta is the difference between consecutive targets → arm position is a clean sine
    phase_now  = 2 * math.pi * t_local       / WIGGLE_PERIOD
    phase_prev = 2 * math.pi * (t_local - 1) / WIGGLE_PERIOD
    delta_val  = amp * (math.sin(phase_now) - math.sin(phase_prev))

    delta = np.zeros(6, dtype=np.float32)
    delta[dof_idx] = delta_val
    return delta, dof_seq_idx, is_new_dof


# ═════════════════════════════════════════════════════════════════════════════
# Node
# ═════════════════════════════════════════════════════════════════════════════

class DummyPolicyNode(Node):

    def __init__(self, mode: str, max_ticks: int):
        super().__init__('dummy_policy')
        self._mode      = mode
        self._max_ticks = max_ticks

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

        self._ik_home   = pulses_to_ik_state(HOME1_PULSES)
        self._ik_state  = self._ik_home.copy()
        self._last_sent = self._ik_home.copy()
        self.get_logger().info(
            f'At home1.  IK: '
            f'th={math.degrees(self._ik_home[0]):+.1f}°  '
            f'p={math.degrees(self._ik_home[1]):+.1f}°  '
            f'r={self._ik_home[2]*100:.1f}cm  '
            f'ud={self._ik_home[3]*100:+.1f}cm')

        # ── State ─────────────────────────────────────────────────────────────
        self._t           = 0
        self._has_image   = False
        self._rb_ok       = 0
        self._rb_fail     = 0
        self._board_lock  = threading.Lock()
        self._state_lock  = threading.Lock()

        # ── ROS ───────────────────────────────────────────────────────────────
        self.create_subscription(Image, '/aurora/rgb/crosshair', self._rgb_cb, 10)
        self._timer = self.create_timer(CMD_INTERVAL, self._control_cb)

        if mode == 'wiggle':
            self.get_logger().info(
                f'Wiggle sequence: '
                + ', '.join(f'{n}' for n, *_ in WIGGLE_SEQUENCE)
                + f'  ({WIGGLE_TICKS / CMD_HZ:.0f} s each, '
                  f'{TOTAL_WIGGLE_TICKS / CMD_HZ:.0f} s total)')
        else:
            self.get_logger().info(
                f'Hold mode — {max_ticks} ticks ({max_ticks / CMD_HZ:.0f} s). '
                'Ctrl-C to stop.')

    def _rgb_cb(self, msg):
        if not self._has_image:
            self.get_logger().info('Camera /aurora/rgb/crosshair live ✓')
            self._has_image = True

    def _control_cb(self):
        if self._t >= self._max_ticks:
            total = self._rb_ok + self._rb_fail
            pct   = 100 * self._rb_ok / total if total else 0
            self.get_logger().info(
                f'Done.  readback {self._rb_ok}/{total} ({pct:.0f}%)  '
                f'cam={"✓" if self._has_image else "✗"}')
            self.destroy_timer(self._timer)
            return

        # ── Servo readback → joint angles ─────────────────────────────────────
        readback = read_servo_pulses(self.board)
        if readback:
            ja = pulses_to_joint_angles(readback)
            self._rb_ok += 1
        else:
            ja = np.zeros(6, dtype=np.float32)
            self._rb_fail += 1
            if self._rb_fail <= 3 or self._rb_fail % 24 == 0:
                self.get_logger().warn(
                    f'Servo readback failed (#{self._rb_fail})')

        # Build 12D observation
        with self._state_lock:
            obs_12d = np.concatenate([self._ik_state.copy(), ja])

        # ── Action ────────────────────────────────────────────────────────────
        if self._mode == 'wiggle':
            delta, dof_seq_idx, is_new_dof = wiggle_action(self._t)

            # Announce each new DOF
            if is_new_dof and dof_seq_idx >= 0:
                name, _, amp, unit = WIGGLE_SEQUENCE[dof_seq_idx]
                amp_display = math.degrees(amp) if unit == 'deg' else (
                    amp * 100 if unit == 'm' else amp)
                unit_display = unit if unit != 'deg' else '°'
                self.get_logger().info(
                    f'[wiggle] DOF {dof_seq_idx + 1}/{len(WIGGLE_SEQUENCE)} — '
                    f'{name}  (±{amp_display:.1f}{unit_display}, '
                    f'{WIGGLE_TICKS / CMD_HZ:.0f} s)')

            # At segment boundaries reset IK state to home to clear float drift
            if is_new_dof and dof_seq_idx > 0:
                with self._state_lock:
                    self._ik_state[:] = self._ik_home

            # Sequence finished → hold position
            if dof_seq_idx == -1:
                delta = np.zeros(6, dtype=np.float32)
        else:
            delta = np.zeros(6, dtype=np.float32)

        # Apply delta and clip to IK bounds
        with self._state_lock:
            self._ik_state = np.clip(
                self._ik_state + delta,
                NORM_LO[:6], NORM_HI[:6])
            ik_new = self._ik_state.copy()
            self._t += 1

        # ── IK solve ──────────────────────────────────────────────────────────
        theta, pitch, radius, up_down, gripper_frac, tilt = ik_new
        result = solve(
            float(theta), float(pitch), float(radius), float(up_down),
            gripper_tilt=float(tilt), gripper=float(gripper_frac))

        if not result.reachable:
            self.get_logger().warn(
                f't={self._t}  IK unreachable — reverting  '
                f'(th={math.degrees(theta):+.1f}° '
                f'p={math.degrees(pitch):+.1f}° '
                f'r={radius*100:.1f}cm '
                f'ud={up_down*100:+.1f}cm)')
            with self._state_lock:
                self._ik_state[:] = self._last_sent
            return

        # ── Send ──────────────────────────────────────────────────────────────
        p = result.pulses
        with self._board_lock:
            self.board.bus_servo_set_position(
                MOVE_DURATION,
                [[6, p[6]], [5, p[5]], [4, p[4]],
                 [3, p[3]], [2, p[2]], [1, p[1]]])

        with self._state_lock:
            self._last_sent[:] = ik_new

        # ── Print 12D observation every second ────────────────────────────────
        if self._t % CMD_HZ == 0:
            self.get_logger().info(
                f't={self._t:4d} '
                f'| IK  '
                f'th={math.degrees(obs_12d[0]):+6.1f}°  '
                f'p={math.degrees(obs_12d[1]):+6.1f}°  '
                f'r={obs_12d[2]*100:+6.2f}cm  '
                f'ud={obs_12d[3]*100:+6.2f}cm  '
                f'gr={obs_12d[4]:.2f}  '
                f'tilt={math.degrees(obs_12d[5]):+5.1f}°'
                f'\n       '
                f'| JA  '
                f'j1={math.degrees(obs_12d[6]):+6.1f}°  '
                f'j2={math.degrees(obs_12d[7]):+6.1f}°  '
                f'j3={math.degrees(obs_12d[8]):+6.1f}°  '
                f'j4={math.degrees(obs_12d[9]):+6.1f}°  '
                f'wr={math.degrees(obs_12d[10]):+6.1f}°  '
                f'gr={obs_12d[11]:.3f}rad'
            )


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Dummy ACT policy — validates the control pipeline without a model.')
    parser.add_argument('--mode', choices=['hold', 'wiggle'], default='hold',
                        help='hold: arm stays at home1.  '
                             'wiggle: cycles through each IK DOF in sequence.')
    parser.add_argument('--ticks', type=int, default=None,
                        help=f'Number of 12 Hz ticks (default: 240 for hold, '
                             f'{TOTAL_WIGGLE_TICKS} for wiggle).')
    args = parser.parse_args()

    if args.ticks is None:
        args.ticks = TOTAL_WIGGLE_TICKS if args.mode == 'wiggle' else 240

    rclpy.init()
    node = DummyPolicyNode(mode=args.mode, max_ticks=args.ticks)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
