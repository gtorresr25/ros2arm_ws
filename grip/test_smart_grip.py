#!/usr/bin/env python3
"""
test_smart_grip.py — Interactive test for smart_grip().

Prompts for an object name, width estimate (or unknown), and estimate source,
then calls smart_grip() and prints the result.  Repeats until you quit.

Usage
-----
    python3 test_smart_grip.py
"""

import sys
import time

from gripper import (
    Board,
    GRIP_LOG_FILE,
    open_gripper,
    release,
    smart_grip,
)

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE   = 1000000

SOURCES = {
    "1": "depth_camera",
    "2": "manual",
    "3": "unknown",
}


def connect() -> Board:
    print(f"\n[INFO] Connecting on {SERIAL_PORT} @ {BAUD_RATE} baud ...")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.\n")
    return board


def ask_width() -> tuple[float | None, str]:
    """Ask for width and source.  Returns (width_m or None, source string)."""
    raw = input("  Estimated width in mm  (or Enter for unknown): ").strip()
    if raw == "":
        return None, "unknown"

    try:
        width_mm = float(raw)
    except ValueError:
        print("  Invalid — treating as unknown.")
        return None, "unknown"

    print("  Estimate source:")
    for k, v in SOURCES.items():
        print(f"    {k}) {v}")
    choice = input("  Choice [1/2/3, default=2]: ").strip() or "2"
    source = SOURCES.get(choice, "manual")

    return width_mm / 1000.0, source


def main() -> None:
    board = connect()

    try:
        while True:
            print("─" * 50)
            obj = input("Object name  (or q to quit): ").strip()
            if obj.lower() == "q":
                break

            width_m, source = ask_width()

            print(f"\n  Place the {obj} between the jaws.")
            input("  Press Enter when ready ...")

            open_gripper(board, duration=1.0)

            result = smart_grip(
                board,
                estimated_width_m = width_m,
                estimate_source   = source,
            )

            print(f"\n  {result}")
            print(f"  Outcome  : {result.outcome}")
            print(f"  Jaw      : {result.jaw_at_contact_mm:.1f} mm")
            if result.contact_offset_mm is not None:
                print(f"  Offset   : {result.contact_offset_mm:+.1f} mm")
            if result.notes:
                for note in result.notes:
                    print(f"  Note     : {note}")

            input("\n  Press Enter to release ...")
            release(board)

    finally:
        print(f"\n[INFO] Log saved to {GRIP_LOG_FILE}")
        release(board)
        board.port.close()
        print("[OK]  Done.")


if __name__ == "__main__":
    main()
