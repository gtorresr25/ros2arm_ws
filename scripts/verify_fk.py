#!/usr/bin/env python3
"""
verify_fk.py — Live FK verification: jog the real arm and watch RViz follow.

Extends joint_jog.py with a ROS2 layer.  Every servo move also publishes
joint angles to /joint_states so robot_state_publisher updates the URDF
model in RViz.  The TF position of link5 is printed after each move,
giving the URDF-computed end-effector position.

Run alongside the simulation in a separate terminal:
  Terminal 1: ros2 launch armpi_ultra_description display.launch.py
  Terminal 2: python3 scripts/verify_fk.py

Controls (same as joint_jog.py):
  A / Z   →  servo 6  base rotation      +/-
  S / X   →  servo 5  shoulder           +/-
  D / C   →  servo 4  elbow              +/-
  F / V   →  servo 3  wrist pitch        +/-
  G / B   →  servo 2  wrist roll         +/-  (not in URDF yet — moves robot only)
  H / N   →  servo 1  gripper            +/-
  1 – 9   →  step size (default 10 pulses)
  Shift+H →  return to home2
  P       →  print current pulses + TF position
  T       →  toggle RViz control (ON = script drives RViz, OFF = sliders drive RViz)
  Esc / Ctrl-C  →  quit
"""

import sys
import os
import math
import threading
import time
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board

from kinematics.transform import _map, joint1_map, joint2_map, joint3_map, joint4_map, joint5_map

# ── Constants ──────────────────────────────────────────────────────────────────

SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 1_000_000
MOVE_DURATION = 0.3
PULSE_MIN     = 0
PULSE_MAX     = 1000

HOME2 = {6: 504, 5: 510, 4: 496, 3: 502, 2: 499, 1: 230}

KEY_MAP = {
    'a': (6, +1), 'z': (6, -1),
    's': (5, +1), 'x': (5, -1),
    'd': (4, +1), 'c': (4, -1),
    'f': (3, +1), 'v': (3, -1),
    'g': (2, +1), 'b': (2, -1),
    'h': (1, +1), 'n': (1, -1),
}

GRIPPER_PULSE_OPEN   = 200
GRIPPER_PULSE_CLOSED = 680

# ── Pulse → joint angles ───────────────────────────────────────────────────────

def pulses_to_joint_states(pulses: dict) -> dict:
    """Convert servo pulse dict to URDF joint name → angle (radians).

    Mapping (servos paired in reverse ID order with URDF chain order):
      S6 → joint1   S5 → joint2   S4 → joint3
      S3 → joint4   S2 → wrist    S1 → gripper_joint
    """
    j1    = math.radians(_map(pulses[6], joint1_map))   # S6 base rotation
    j2    = -math.radians(_map(pulses[5], joint2_map)) - math.pi / 2  # S5 shoulder (flipped - 90°)
    j3    = math.radians(_map(pulses[4], joint3_map))   # S4 elbow
    j4    = -math.radians(_map(pulses[3], joint4_map)) - math.pi / 2  # S3 wrist pitch (flipped - 90°)
    wrist = math.radians(_map(pulses[2], joint5_map))  # S2 wrist roll

    t = pulses[1]
    gripper = (GRIPPER_PULSE_CLOSED - t) / (GRIPPER_PULSE_CLOSED - GRIPPER_PULSE_OPEN) * 0.785
    gripper = max(0.0, min(0.785, gripper))

    return {
        'joint1':        j1,
        'joint2':        j2,
        'joint3':        j3,
        'joint4':        j4,
        'wrist':         wrist,
        'gripper_joint': gripper,
    }

# ── ROS2 node ──────────────────────────────────────────────────────────────────

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('verify_fk')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def publish(self, angles: dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(angles.keys())
        msg.position = list(angles.values())
        self.pub.publish(msg)

    def ee_position(self) -> str:
        """Return formatted EE position string from TF, or empty string on failure."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'link5', rclpy.time.Time()
            )
            t = tf.transform.translation
            return f"EE → x={t.x*1000:6.1f}mm  y={t.y*1000:6.1f}mm  z={t.z*1000:6.1f}mm"
        except Exception:
            return "EE → (waiting for TF…)"

# ── Hardware helpers ───────────────────────────────────────────────────────────

def connect() -> Board:
    print(f"[INFO] Connecting on {SERIAL_PORT} …")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.\n")
    return board


def engage(board: Board):
    for sid in [1, 2, 3, 4, 5, 6]:
        board.bus_servo_enable_torque(sid, False)
        time.sleep(0.02)


def read_key() -> str:
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch += sys.stdin.read(2)
    return ch


def fmt(pulses: dict) -> str:
    return "  ".join(f"S{sid}={pulses[sid]:4d}" for sid in [6, 5, 4, 3, 2, 1])

# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    # Start ROS2 in a background thread
    rclpy.init()
    node = JointStatePublisher()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Connect to robot
    board = connect()
    engage(board)

    pulses = dict(HOME2)

    print("Moving to home2 …")
    board.bus_servo_set_position(2.0, [[sid, p] for sid, p in pulses.items()])
    time.sleep(2.5)

    # Publish initial pose
    node.publish(pulses_to_joint_states(pulses))

    step = 10

    print("\nReady.  Controls:")
    print("  A/Z  S/X  D/C  F/V  G/B  H/N  →  S6 S5 S4 S3 S2 S1  +/-")
    print("  1-9 step   Shift+H home   P print   Esc quit\n")
    print(fmt(pulses))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            key = read_key()

            if key in ('\x1b', '\x03'):
                break

            elif key in KEY_MAP:
                sid, direction = KEY_MAP[key]
                new_pulse = max(PULSE_MIN, min(PULSE_MAX, pulses[sid] + direction * step))
                pulses[sid] = new_pulse
                board.bus_servo_set_position(MOVE_DURATION, [[sid, new_pulse]])

                # Publish updated joint states to RViz
                node.publish(pulses_to_joint_states(pulses))

                ee = node.ee_position()
                print(f"\r{fmt(pulses)}   {ee}   ", end='', flush=True)

            elif key in '123456789':
                step = int(key) * 10
                print(f"\r{fmt(pulses)}   step={step}   ", end='', flush=True)

            elif key == 'H':
                pulses = dict(HOME2)
                board.bus_servo_set_position(2.0, [[sid, p] for sid, p in pulses.items()])
                time.sleep(2.5)
                node.publish(pulses_to_joint_states(pulses))
                ee = node.ee_position()
                print(f"\r{fmt(pulses)}   {ee}   homed   ", end='', flush=True)

            elif key == 'p':
                ee = node.ee_position()
                print(f"\r{fmt(pulses)}   {ee}   ", end='', flush=True)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n[INFO] Quit.")
        board.port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()