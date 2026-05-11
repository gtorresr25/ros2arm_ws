#!/usr/bin/env python3
"""
joint_jog.py — Jog individual joints by pulse step.

  A / Z   →  servo 6  base rotation    +/-
  S / X   →  servo 5  shoulder         +/-
  D / C   →  servo 4  elbow            +/-
  F / V   →  servo 3  wrist pitch      +/-
  G / B   →  servo 2  wrist roll       +/-

  1 – 9   →  step size (default 10 pulses)
  H (shift)  →  return to home2
  P       →  print current pulses
  Esc / Ctrl-C  →  quit

Run:
  python3 scripts/joint_jog.py
"""

import sys, tty, termios, select, time

SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board

SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 1_000_000
MOVE_DURATION = 0.3          # seconds per command
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


def read_key():
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch += sys.stdin.read(2)
    return ch


def connect():
    print(f"[INFO] Connecting on {SERIAL_PORT} …")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.\n")
    return board


def engage(board):
    for sid in [1, 2, 3, 4, 5, 6]:
        board.bus_servo_enable_torque(sid, False)
        time.sleep(0.02)


def send(board, servo_id, pulse):
    board.bus_servo_set_position(MOVE_DURATION, [[servo_id, pulse]])


def fmt(pulses):
    return "  ".join(f"S{sid}={pulses[sid]:4d}" for sid in [6, 5, 4, 3, 2, 1])


def main():
    board = connect()
    engage(board)

    pulses = dict(HOME2)

    print("Moving to home2 …")
    board.bus_servo_set_position(2.0, [[sid, p] for sid, p in pulses.items()])
    time.sleep(2.5)

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
                send(board, sid, new_pulse)
                print(f"\r{fmt(pulses)}   ", end='', flush=True)

            elif key in '123456789':
                step = int(key) * 10
                print(f"\r{fmt(pulses)}   step={step}   ", end='', flush=True)

            elif key == 'H':
                pulses = dict(HOME2)
                board.bus_servo_set_position(2.0, [[sid, p] for sid, p in pulses.items()])
                time.sleep(2.5)
                print(f"\r{fmt(pulses)}   homed        ", end='', flush=True)

            elif key == 'p':
                print(f"\r{fmt(pulses)}   ", end='', flush=True)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n[INFO] Quit.")
        board.port.close()


if __name__ == "__main__":
    main()