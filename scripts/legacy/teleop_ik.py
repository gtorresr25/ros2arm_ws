#!/usr/bin/env python3
"""
teleop_ik.py — Keyboard teleop using IK control knobs.

Controls:
  W / S      radius      forward / backward (along camera axis)
  R / F      up_down     up / down (perpendicular to camera axis)
  Q / E      pitch       tilt up / tilt down
  A / D      theta       base rotate left / right
  H          go to home1
  1 – 9      step size multiplier  (1 = smallest, 9 = largest)
  P          print current state
  Esc / Ctrl-C   quit

Requires:
  colcon build --packages-select kinematics
  source install/setup.bash
"""

import sys
import os
import math
import time
import tty
import termios
import select
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ── SDK ───────────────────────────────────────────────────────────────────────
SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board

# ── IK ────────────────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/andres/ros2arm_ws/install/kinematics/lib/python3/dist-packages")
from kinematics.ik import solve, D_BASE, L1, L2, L_TCP
from kinematics.transform import _map, joint1_map, clamp_pulses

# ── Hardware ──────────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 1_000_000
MOVE_DURATION = 0.25

# ── Home positions ────────────────────────────────────────────────────────────
HOME1_PULSES = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}

_J2_MAP = [0, 1000, 500,   30, -210, -90]
_J3_MAP = [0, 1000, 500, -120,  120,   0]
_J4_MAP = [0, 1000, 500,   30, -210, -90]
_J1_MAP = [0, 1000, 500, -120,  120,   0]

def _fwd(v, m):
    return ((v - m[2]) / (m[1] - m[0])) * (m[4] - m[3]) + m[5]

def pulses_to_state(p):
    """Convert servo pulse dict → (theta, pitch, radius, up_down)."""
    theta = math.radians(_fwd(p[6], _J1_MAP))
    j2    = -math.radians(_fwd(p[5], _J2_MAP)) - math.pi / 2
    j3    =  math.radians(_fwd(p[4], _J3_MAP))
    j4    = -math.radians(_fwd(p[3], _J4_MAP)) - math.pi / 2

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

    return theta, pitch, radius, up_down

# ── ROS2 joint state publisher ────────────────────────────────────────────────

class JointPublisher(Node):
    def __init__(self):
        super().__init__('teleop_ik')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

    def publish(self, joints: dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(joints.keys())
        msg.position = list(joints.values())
        self.pub.publish(msg)

# ── Step sizes ────────────────────────────────────────────────────────────────
BASE_STEPS = {
    'theta':   math.radians(3),   # 3° per unit
    'pitch':   math.radians(3),   # 3° per unit
    'radius':  0.005,             # 5 mm per unit
    'up_down': 0.005,             # 5 mm per unit
}

# ── Key reading ───────────────────────────────────────────────────────────────
def read_key():
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch += sys.stdin.read(2)
    return ch

# ── Display ───────────────────────────────────────────────────────────────────
def fmt_state(theta, pitch, radius, up_down, step, reachable):
    r_str = "OK" if reachable else "!!"
    return (f"[{r_str}]  "
            f"θ={math.degrees(theta):+6.1f}°  "
            f"pitch={math.degrees(pitch):+6.1f}°  "
            f"radius={radius*100:5.1f}cm  "
            f"up={up_down*100:+5.1f}cm  "
            f"step×{step}")

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    # Start ROS2 in background thread
    rclpy.init()
    ros_node = JointPublisher()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

    print(f"[INFO] Connecting on {SERIAL_PORT} …")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.")

    # Engage servos
    for sid in range(1, 7):
        board.bus_servo_enable_torque(sid, False)
        time.sleep(0.02)

    # Move to home1
    print("Moving to home1 …")
    board.bus_servo_set_position(2.0, [[sid, p] for sid, p in HOME1_PULSES.items()])
    time.sleep(2.5)

    # Derive starting state from home1 pulses
    theta, pitch, radius, up_down = pulses_to_state(HOME1_PULSES)
    gripper_tilt = 0.0
    gripper      = 0.0   # 0.0 = fully open, 1.0 = fully closed
    step = 1

    # Publish initial pose to RViz
    init_result = solve(theta, pitch, radius, up_down, gripper=gripper)
    ros_node.publish(init_result.joints)

    print("\nReady.")
    print("  W/S  up_down    Q/E  radius     R/F  pitch    A/D  base")
    print("  Z    open       X    close      H    home1    1-9  step")
    print("  P    print      Esc  quit\n")
    print(fmt_state(theta, pitch, radius, up_down, step, True))

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            key = read_key()

            if key in ('\x1b', '\x03'):
                break

            elif key in '123456789':
                step = int(key)
                print(f"\r{fmt_state(theta, pitch, radius, up_down, step, True)}   ", end='', flush=True)
                continue

            elif key == 'H':
                board.bus_servo_set_position(2.0, [[sid, p] for sid, p in HOME1_PULSES.items()])
                time.sleep(2.5)
                theta, pitch, radius, up_down = pulses_to_state(HOME1_PULSES)
                home_result = solve(theta, pitch, radius, up_down)
                ros_node.publish(home_result.joints)
                print(f"\r{fmt_state(theta, pitch, radius, up_down, step, True)}  [home1]  ", end='', flush=True)
                continue

            elif key == 'p':
                print(f"\r{fmt_state(theta, pitch, radius, up_down, step, True)}   ", end='', flush=True)
                continue

            elif key in ('z', 'x'):
                gripper = max(0.0, min(1.0, gripper + (-0.1 if key == 'z' else +0.1)))
                p1 = int(200 + gripper * (680 - 200))
                board.bus_servo_set_position(MOVE_DURATION, [[1, p1]])
                result = solve(theta, pitch, radius, up_down, gripper_tilt=gripper_tilt, gripper=gripper)
                ros_node.publish(result.joints)
                print(f"\r{fmt_state(theta, pitch, radius, up_down, step, True)}  grip={gripper*100:.0f}%   ", end='', flush=True)
                continue

            # Knob deltas
            delta = {
                'w': ('up_down', +1), 's': ('up_down', -1),
                'e': ('radius',  +1), 'q': ('radius',  -1),
                'r': ('pitch',   +1), 'f': ('pitch',   -1),
                'a': ('theta',   +1), 'd': ('theta',   -1),
            }.get(key)

            if delta is None:
                continue

            knob, sign = delta
            if   knob == 'radius':  radius      += sign * step * BASE_STEPS['radius']
            elif knob == 'up_down': up_down     += sign * step * BASE_STEPS['up_down']
            elif knob == 'pitch':   pitch       += sign * step * BASE_STEPS['pitch']
            elif knob == 'theta':   theta       += sign * step * BASE_STEPS['theta']

            result = solve(theta, pitch, radius, up_down, gripper_tilt=gripper_tilt, gripper=gripper)

            if result.reachable:
                p = result.pulses
                board.bus_servo_set_position(
                    MOVE_DURATION,
                    [[6, p[6]], [5, p[5]], [4, p[4]], [3, p[3]], [2, p[2]]]
                )
                ros_node.publish(result.joints)
            else:
                # Revert the move
                if   knob == 'radius':  radius      -= sign * step * BASE_STEPS['radius']
                elif knob == 'up_down': up_down     -= sign * step * BASE_STEPS['up_down']
                elif knob == 'pitch':   pitch       -= sign * step * BASE_STEPS['pitch']
                elif knob == 'theta':   theta       -= sign * step * BASE_STEPS['theta']

            print(f"\r{fmt_state(theta, pitch, radius, up_down, step, result.reachable)}   ",
                  end='', flush=True)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n[INFO] Quit.")
        board.port.close()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()