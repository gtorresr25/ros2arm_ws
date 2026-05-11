#!/usr/bin/env python3
"""
fk_measure.py — Move arm to named poses and collect physical measurements.

Use a ruler/tape to measure the gripper tip position (X, Y, Z) relative
to the arm's rotation axis at the base plate.

  python3 scripts/fk_measure.py
"""

import sys
import time
import json

SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE   = 1_000_000

# Poses to test — add or edit freely
# Servo order: [S1_gripper, S2_wrist_roll, S3_wrist_pitch, S4_elbow, S5_shoulder, S6_base]
POSES = {
    "home2_straight_up": {
        "pulses": {1: 230, 2: 499, 3: 502, 4: 496, 5: 510, 6: 504},
        "note": "All servos near 500 — arm should point straight up",
    },
    "home1": {
        "pulses": {1: 230, 2: 498, 3: 111, 4: 824, 5: 603, 6: 504},
        "note": "home1 saved pose",
    },
    "reach_forward": {
        "pulses": {1: 230, 2: 498, 3: 350, 4: 650, 5: 600, 6: 504},
        "note": "Arm reaching roughly forward and level",
    },
}

DURATION = 3.0   # seconds to move to each pose


def connect():
    print(f"[INFO] Connecting on {SERIAL_PORT} …")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.\n")
    return board


def engage(board):
    for sid in [1, 2, 3, 4, 5, 6]:
        board.bus_servo_enable_torque(sid, False)   # False = LOAD
        time.sleep(0.02)


def move_to(board, pulses: dict, duration: float):
    cmd = [[sid, pulse] for sid, pulse in pulses.items()]
    board.bus_servo_set_position(duration, cmd)


def main():
    board = connect()
    engage(board)

    results = {}

    for name, pose in POSES.items():
        print(f"─── {name} ─────────────────────────────────")
        print(f"    {pose['note']}")
        print(f"    Pulses: {pose['pulses']}")

        ans = input("    Move to this pose? [Y/n] ").strip().lower()
        if ans == "n":
            print("    Skipped.\n")
            continue

        move_to(board, pose["pulses"], DURATION)
        time.sleep(DURATION + 0.5)

        print()
        print("    Measure the gripper tip position (mm) relative to the arm")
        print("    rotation axis at the base plate.")
        print("    X = forward/back  Y = left/right  Z = up/down")
        print()

        try:
            x = float(input("    X (mm): "))
            y = float(input("    Y (mm): "))
            z = float(input("    Z (mm): "))
        except ValueError:
            print("    Invalid — skipping measurement.")
            print()
            continue

        results[name] = {
            "pulses":      pose["pulses"],
            "measured_mm": {"x": x, "y": y, "z": z},
        }
        print(f"    Recorded: x={x} y={y} z={z} mm\n")

    board.port.close()

    if results:
        out = "/home/andres/ros2arm_ws/scripts/fk_measurements.json"
        with open(out, "w") as f:
            json.dump(results, f, indent=2)
        print(f"[OK]  Saved to {out}")
    else:
        print("[INFO] No measurements recorded.")


if __name__ == "__main__":
    main()