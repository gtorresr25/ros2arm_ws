#!/usr/bin/env python3
"""
teleop_ik_tcp.py — Keyboard teleop, TCP-pitch mode.

Pitch rotates the tool AT the TCP — the tip stays fixed, arm reconfigures.
r_tcp and z_tcp move the tip in world coordinates.

Controls:
  W / S      r_tcp       reach forward / backward  (world horizontal)
  R / F      z_tcp       move up / down             (world height)
  Q / E      pitch       tilt up / tilt down        (rotates at TCP)
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
import math
import time
import tty
import termios
import select

# ── SDK ───────────────────────────────────────────────────────────────────────
SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board

# ── IK ────────────────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/andres/ros2arm_ws/install/kinematics/lib/python3/dist-packages")
from kinematics.ik_tcp import solve, D_BASE, L1, L2, L_TCP

# ── Hardware ──────────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 1_000_000
MOVE_DURATION = 0.25

# ── Home positions ────────────────────────────────────────────────────────────
HOME1_PULSES = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}

_J1_MAP = [0, 1000, 500, -120,  120,   0]
_J2_MAP = [0, 1000, 500,   30, -210, -90]
_J3_MAP = [0, 1000, 500, -120,  120,   0]
_J4_MAP = [0, 1000, 500,   30, -210, -90]

def _fwd(v, m):
    return ((v - m[2]) / (m[1] - m[0])) * (m[4] - m[3]) + m[5]

def pulses_to_state(p):
    """Convert servo pulse dict → (theta, pitch, r_tcp, z_tcp)."""
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
    r_tcp   = wrist_r + L_TCP * math.cos(pitch)
    z_tcp   = wrist_z + L_TCP * math.sin(pitch)

    return theta, pitch, r_tcp, z_tcp

# ── Step sizes ────────────────────────────────────────────────────────────────
BASE_STEPS = {
    'theta': math.radians(3),   # 3° per unit
    'pitch': math.radians(3),   # 3° per unit
    'r_tcp': 0.005,             # 5 mm per unit
    'z_tcp': 0.005,             # 5 mm per unit
}

# ── Key reading ───────────────────────────────────────────────────────────────
def read_key():
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch += sys.stdin.read(2)
    return ch

# ── Display ───────────────────────────────────────────────────────────────────
def fmt_state(theta, pitch, r_tcp, z_tcp, step, reachable):
    r_str = "OK" if reachable else "!!"
    return (f"[{r_str}]  "
            f"θ={math.degrees(theta):+6.1f}°  "
            f"pitch={math.degrees(pitch):+6.1f}°  "
            f"r={r_tcp*100:5.1f}cm  "
            f"z={z_tcp*100:5.1f}cm  "
            f"step×{step}")

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(f"[INFO] Connecting on {SERIAL_PORT} …")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.")

    for sid in range(1, 7):
        board.bus_servo_enable_torque(sid, False)
        time.sleep(0.02)

    print("Moving to home1 …")
    board.bus_servo_set_position(2.0, [[sid, p] for sid, p in HOME1_PULSES.items()])
    time.sleep(2.5)

    theta, pitch, r_tcp, z_tcp = pulses_to_state(HOME1_PULSES)
    gripper_tilt = 0.0
    step = 1

    print("\nReady.")
    print("  W/S  r_tcp (reach)    R/F  z_tcp (height)    Q/E  pitch (at TCP)    A/D  base")
    print("  H    home1            1-9  step              P    print             Esc  quit\n")
    print(fmt_state(theta, pitch, r_tcp, z_tcp, step, True))

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
                print(f"\r{fmt_state(theta, pitch, r_tcp, z_tcp, step, True)}   ", end='', flush=True)
                continue

            elif key == 'H':
                board.bus_servo_set_position(2.0, [[sid, p] for sid, p in HOME1_PULSES.items()])
                time.sleep(2.5)
                theta, pitch, r_tcp, z_tcp = pulses_to_state(HOME1_PULSES)
                print(f"\r{fmt_state(theta, pitch, r_tcp, z_tcp, step, True)}  [home1]  ", end='', flush=True)
                continue

            elif key == 'p':
                print(f"\r{fmt_state(theta, pitch, r_tcp, z_tcp, step, True)}   ", end='', flush=True)
                continue

            delta = {
                'w': ('r_tcp',  +1), 's': ('r_tcp',  -1),
                'r': ('z_tcp',  +1), 'f': ('z_tcp',  -1),
                'q': ('pitch',  +1), 'e': ('pitch',  -1),
                'a': ('theta',  +1), 'd': ('theta',  -1),
            }.get(key)

            if delta is None:
                continue

            knob, sign = delta
            if   knob == 'r_tcp':  r_tcp += sign * step * BASE_STEPS['r_tcp']
            elif knob == 'z_tcp':  z_tcp += sign * step * BASE_STEPS['z_tcp']
            elif knob == 'pitch':  pitch += sign * step * BASE_STEPS['pitch']
            elif knob == 'theta':  theta += sign * step * BASE_STEPS['theta']

            result = solve(theta, pitch, r_tcp, z_tcp, gripper_tilt=gripper_tilt)

            if result.reachable:
                p = result.pulses
                board.bus_servo_set_position(
                    MOVE_DURATION,
                    [[6, p[6]], [5, p[5]], [4, p[4]], [3, p[3]], [2, p[2]]]
                )
            else:
                if   knob == 'r_tcp':  r_tcp -= sign * step * BASE_STEPS['r_tcp']
                elif knob == 'z_tcp':  z_tcp -= sign * step * BASE_STEPS['z_tcp']
                elif knob == 'pitch':  pitch -= sign * step * BASE_STEPS['pitch']
                elif knob == 'theta':  theta -= sign * step * BASE_STEPS['theta']

            print(f"\r{fmt_state(theta, pitch, r_tcp, z_tcp, step, result.reachable)}   ",
                  end='', flush=True)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n[INFO] Quit.")
        board.port.close()


if __name__ == "__main__":
    main()