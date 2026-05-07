#!/usr/bin/env python3
"""
ArmPi Ultra — home position manager (no ROS required).

Commands
--------
  python3 arm_homes.py record <name>
      Read current servo positions from the arm and save them as <name>.
      Example:  python3 arm_homes.py record home1
                python3 arm_homes.py record home2

  python3 arm_homes.py release
      Release torque on all servos so you can physically position the arm
      by hand. Press Enter when done to re-engage torque and record the pose.

  python3 arm_homes.py goto <name> [duration]
      Move all servos to a saved home position.
      duration is optional, in seconds (default 2.0 — slow and safe).
      Example:  python3 arm_homes.py goto home1
                python3 arm_homes.py goto home2 3.0

  python3 arm_homes.py list
      Show all saved home positions.

Workflow for defining home2 (straight up)
------------------------------------------
  1. python3 arm_homes.py release     # arm goes limp — position it by hand
  2. (physically move arm to straight-up pose)
  3. Press Enter                       # torque re-engages, positions recorded
  4. python3 arm_homes.py list         # verify
"""

import sys
import time
import json
import os

# ── SDK path ─────────────────────────────────────────────────────────────────
SDK_PATH = "/home/andres/ros2arm_ws/ArmPi_Ultra_Resources/Source Code/ROS2/src/driver/ros_robot_controller/ros_robot_controller"
sys.path.insert(0, SDK_PATH)

try:
    from ros_robot_controller_sdk import Board
except ImportError as e:
    print(f"[ERROR] Could not import SDK: {e}")
    sys.exit(1)

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT    = "/dev/ttyUSB0"
BAUD_RATE      = 1000000
SERVO_IDS      = [1, 2, 3, 4, 5, 6]
HOMES_FILE     = os.path.join(os.path.dirname(__file__), "home_positions.json")
DEFAULT_DURATION = 5.0   # seconds — slow and safe


# ── SDK quirk note ────────────────────────────────────────────────────────────
# bus_servo_enable_torque(id, True)  → 0x0B = UNLOAD (releases, arm goes limp)
# bus_servo_enable_torque(id, False) → 0x0C = LOAD   (engages, arm holds)

def connect() -> Board:
    print(f"[INFO] Connecting on {SERIAL_PORT} …")
    try:
        board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
        board.enable_reception(True)
        time.sleep(0.3)
        print("[OK]  Connected.")
        return board
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)


def load_homes() -> dict:
    if os.path.exists(HOMES_FILE):
        with open(HOMES_FILE) as f:
            return json.load(f)
    return {}


def save_homes(homes: dict) -> None:
    with open(HOMES_FILE, "w") as f:
        json.dump(homes, f, indent=2)


def read_positions(board: Board) -> dict | None:
    """Read current positions for all servos. Returns {id: position} or None on failure."""
    positions = {}
    for sid in SERVO_IDS:
        result = board.bus_servo_read_position(sid)
        if result is None:
            print(f"  [WARN] Servo {sid} did not respond.")
            return None
        positions[sid] = result[0]
    return positions


def engage_all(board: Board) -> None:
    for sid in SERVO_IDS:
        board.bus_servo_enable_torque(sid, False)  # False → 0x0C = LOAD
        time.sleep(0.02)


def release_all(board: Board) -> None:
    for sid in SERVO_IDS:
        board.bus_servo_enable_torque(sid, True)   # True → 0x0B = UNLOAD
        time.sleep(0.02)


# ── Commands ──────────────────────────────────────────────────────────────────

def cmd_record(name: str) -> None:
    board = connect()
    print(f"\nReading current servo positions …")
    positions = read_positions(board)
    if positions is None:
        print("[ERROR] Could not read all servos. Is the arm powered on?")
        board.port.close()
        sys.exit(1)

    print(f"\n  Current positions:")
    for sid in SERVO_IDS:
        print(f"    Servo {sid}: {positions[sid]}")

    homes = load_homes()
    if name in homes:
        answer = input(f"\nOverwrite existing '{name}'? [y/N] ").strip().lower()
        if answer != "y":
            print("Cancelled.")
            board.port.close()
            return

    homes[name] = {str(sid): positions[sid] for sid in SERVO_IDS}
    save_homes(homes)
    print(f"\n[OK]  Saved as '{name}' → {HOMES_FILE}")
    board.port.close()


def cmd_release() -> None:
    """Release torque, let user position arm by hand, then record."""
    name = input("Name to save this pose as (e.g. home2): ").strip()
    if not name:
        print("[ERROR] Name cannot be empty.")
        sys.exit(1)

    board = connect()
    print(f"\n[INFO] Releasing torque on all servos — arm will go limp.")
    print("       Move the arm to the desired position by hand.")
    release_all(board)

    input("\nPress Enter when the arm is in position …")

    print("[INFO] Re-engaging torque …")
    engage_all(board)
    time.sleep(0.3)

    print("[INFO] Reading position …")
    positions = read_positions(board)
    if positions is None:
        print("[ERROR] Could not read positions after re-engaging torque.")
        board.port.close()
        sys.exit(1)

    print(f"\n  Recorded positions:")
    for sid in SERVO_IDS:
        print(f"    Servo {sid}: {positions[sid]}")

    homes = load_homes()
    homes[name] = {str(sid): positions[sid] for sid in SERVO_IDS}
    save_homes(homes)
    print(f"\n[OK]  Saved as '{name}' → {HOMES_FILE}")
    board.port.close()


def cmd_goto(name: str, duration: float) -> None:
    homes = load_homes()
    if name not in homes:
        print(f"[ERROR] No home position named '{name}'.")
        print(f"  Known positions: {list(homes.keys()) or '(none)'}")
        sys.exit(1)

    positions = homes[name]
    move_cmd = [[int(sid), positions[str(sid)]] for sid in SERVO_IDS]

    print(f"\n  Target positions ({name}):")
    for sid in SERVO_IDS:
        print(f"    Servo {sid}: {positions[str(sid)]}")

    answer = input(f"\nMove arm to '{name}' over {duration}s? [y/N] ").strip().lower()
    if answer != "y":
        print("Cancelled.")
        return

    board = connect()
    print(f"\n[INFO] Engaging torque …")
    engage_all(board)
    time.sleep(0.1)

    print(f"[INFO] Moving to '{name}' …")
    board.bus_servo_set_position(duration, move_cmd)
    time.sleep(duration + 0.5)

    print(f"[OK]  Done.")
    board.port.close()


def cmd_list() -> None:
    homes = load_homes()
    if not homes:
        print("No home positions saved yet.")
        print(f"  File: {HOMES_FILE}")
        return

    print(f"\nSaved home positions ({HOMES_FILE}):\n")
    for name, positions in homes.items():
        vals = "  ".join(f"S{sid}={positions[str(sid)]}" for sid in SERVO_IDS)
        print(f"  {name}:  {vals}")


# ── Entry point ───────────────────────────────────────────────────────────────

def usage() -> None:
    print(__doc__)
    sys.exit(1)


def main() -> None:
    args = sys.argv[1:]

    if not args:
        usage()

    cmd = args[0]

    if cmd == "record":
        if len(args) < 2:
            print("Usage: python3 arm_homes.py record <name>")
            sys.exit(1)
        cmd_record(args[1])

    elif cmd == "release":
        cmd_release()

    elif cmd == "goto":
        if len(args) < 2:
            print("Usage: python3 arm_homes.py goto <name> [duration]")
            sys.exit(1)
        duration = float(args[2]) if len(args) >= 3 else DEFAULT_DURATION
        cmd_goto(args[1], duration)

    elif cmd == "list":
        cmd_list()

    else:
        print(f"[ERROR] Unknown command '{cmd}'")
        usage()


if __name__ == "__main__":
    main()
