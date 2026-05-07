#!/usr/bin/env python3
"""
ArmPi Ultra — minimal connection and servo test (no ROS required).

Steps this script performs:
  1. Connect to the STM32 over USB serial
  2. Read battery voltage   (safe: no movement)
  3. Read position of each servo 1-6   (safe: no movement)
  4. Ask for confirmation before moving
  5. Move servo 1 slowly to the centre position (500) then back

Important SDK quirk
-------------------
bus_servo_enable_torque(servo_id, enable=True)  → sends 0x0B = UNLOAD (releases torque, arm goes limp)
bus_servo_enable_torque(servo_id, enable=False) → sends 0x0C = LOAD  (engages torque, arm holds position)
The argument name in the SDK is inverted relative to its meaning.
"""

import sys
import time

# ── Add the SDK to the import path ──────────────────────────────────────────
SDK_PATH = "/home/andres/ros2arm_ws/ArmPi_Ultra_Resources/Source Code/ROS2/src/driver/ros_robot_controller/ros_robot_controller"
sys.path.insert(0, SDK_PATH)

try:
    from ros_robot_controller_sdk import Board
except ImportError as e:
    print(f"[ERROR] Could not import SDK: {e}")
    print(f"  Expected SDK at: {SDK_PATH}")
    sys.exit(1)

# ── Configuration ────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"   # change to /dev/ttyACM0 if ttyUSB0 is not found
BAUD_RATE     = 1000000
SERVO_IDS     = [1, 2, 3, 4, 5, 6]   # adjust if your arm has fewer joints
CENTRE_POS    = 500               # mid-range of 0-1000 pulse scale
MOVE_DURATION = 2.0               # seconds — slow and safe


def connect(port: str) -> Board:
    print(f"[INFO] Connecting to STM32 on {port} @ {BAUD_RATE} baud …")
    try:
        board = Board(device=port, baudrate=BAUD_RATE)
        board.enable_reception(True)
        time.sleep(0.3)           # let the receive thread settle
        print("[OK]  Serial port opened.")
        return board
    except Exception as e:
        print(f"[ERROR] Could not open {port}: {e}")
        print("  Tip: check 'ls /dev/ttyUSB*' and 'ls /dev/ttyACM*'")
        print("       and make sure your user is in the 'dialout' group:")
        print("       sudo usermod -aG dialout $USER")
        sys.exit(1)


def read_battery(board: Board) -> None:
    print("\n── Battery voltage ──────────────────────────────────────────")
    # The board pushes battery data continuously; give it a moment to arrive
    time.sleep(0.5)
    voltage = board.get_battery()
    if voltage is not None:
        print(f"  Voltage: {voltage} mV  ({voltage/1000:.2f} V)")
    else:
        print("  [WARN] No battery data received yet (normal if STM32 just booted).")


def read_servo_positions(board: Board) -> dict:
    print("\n── Current servo positions ──────────────────────────────────")
    positions = {}
    for sid in SERVO_IDS:
        result = board.bus_servo_read_position(sid)
        if result is not None:
            pos = result[0]
            positions[sid] = pos
            print(f"  Servo {sid}: {pos}  (range 0–1000)")
        else:
            print(f"  Servo {sid}: [no response — servo may not exist or be powered off]")
    return positions


def move_servo_test(board: Board, servo_id: int = 1) -> None:
    print(f"\n── Move test on servo {servo_id} ────────────────────────────")

    # Engage torque before moving  (enable=False → 0x0C = LOAD)
    print(f"  Engaging torque on servo {servo_id} …")
    board.bus_servo_enable_torque(servo_id, False)   # False = engage (see header note)
    time.sleep(0.1)

    print(f"  Moving servo {servo_id} → position {CENTRE_POS}  over {MOVE_DURATION}s …")
    board.bus_servo_set_position(MOVE_DURATION, [[servo_id, CENTRE_POS]])
    time.sleep(MOVE_DURATION + 0.5)

    result = board.bus_servo_read_position(servo_id)
    if result is not None:
        print(f"  Position after move: {result[0]}")

    print(f"  Moving servo {servo_id} back to original position …")
    original = read_servo_positions(board).get(servo_id, CENTRE_POS)
    board.bus_servo_set_position(MOVE_DURATION, [[servo_id, original]])
    time.sleep(MOVE_DURATION + 0.5)

    # Release torque (enable=True → 0x0B = UNLOAD — arm relaxes)
    print(f"  Releasing torque on servo {servo_id} …")
    board.bus_servo_enable_torque(servo_id, True)    # True = release (see header note)
    print("  Done.")


def main() -> None:
    board = connect(SERIAL_PORT)

    read_battery(board)
    read_servo_positions(board)

    print("\n────────────────────────────────────────────────────────────")
    answer = input("Move servo 1 to centre position (500) as a motion test? [y/N] ").strip().lower()
    if answer == "y":
        move_servo_test(board, servo_id=1)
    else:
        print("[INFO] Motion test skipped.")

    print("\n[INFO] Test complete. Closing serial port.")
    board.port.close()


if __name__ == "__main__":
    main()
