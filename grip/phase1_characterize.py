#!/usr/bin/env python3
"""
phase1_characterize.py — Phase 1: Gripper characterization.

Moves the gripper to a series of known pulse positions and prompts you to
measure the actual jaw gap with calipers.  Compares your measurements against
the theoretical values from the four-bar linkage model in gripper.py.

Results are saved to phase1_results.json for later reference.

Usage
-----
    python3 phase1_characterize.py

What to bring
-------------
  - Calipers (or a ruler if calipers aren't available)
  - The arm powered on and connected via /dev/ttyUSB0
  - A clear workspace — the gripper will move freely, no objects needed

Steps this script runs
----------------------
  1. Print the theoretical pulse → width table
  2. Connect to the arm
  3. For each test pulse: move, wait, ask you to measure, record
  4. Print a comparison table (theoretical vs measured vs error)
  5. Save results to phase1_results.json
  6. Flag any systematic offset that might need correcting in gripper.py
"""

import json
import os
import sys
import time

from gripper import (
    Board,
    GRIPPER_ID,
    PULSE_OPEN,
    PULSE_MAX,
    _move,
    _read_position,
    _read_temp,
    open_gripper,
    print_position_map,
    pulse_to_width,
    read_gripper_state,
)

SERIAL_PORT  = "/dev/ttyUSB0"
BAUD_RATE    = 1000000
RESULTS_FILE = os.path.join(os.path.dirname(__file__), "phase1_results.json")

# Pulses to test — spread across the usable range.
# Skips the very end (PULSE_MAX) for the calibration pass; we do a final
# open at the end to leave the arm safe.
TEST_PULSES = [200, 260, 320, 380, 440, 500, 560, 620, 680]


def connect() -> Board:
    print(f"\n[INFO] Connecting on {SERIAL_PORT} @ {BAUD_RATE} baud ...")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.")
    return board


def prompt_measurement(pulse: int, theoretical_mm: float) -> float | None:
    """Ask the user to measure the jaw gap and return the value in mm."""
    print(f"\n  Theoretical jaw width at pulse {pulse}: {theoretical_mm:.1f} mm")
    print( "  ──────────────────────────────────────────────────────")
    print( "  Measure the inner gap between the jaw tips with calipers.")
    raw = input("  Enter measured width in mm  (or Enter to skip): ").strip()
    if raw == "":
        return None
    try:
        return float(raw)
    except ValueError:
        print("  [WARN] Could not parse value — skipping this point.")
        return None


def run_characterization(board: Board) -> list[dict]:
    """Step through each test pulse, record measurements, return results."""
    results = []

    print("\n" + "═" * 58)
    print("  PHASE 1 — GRIPPER CHARACTERIZATION")
    print("═" * 58)
    print("  The gripper will move to each test pulse in sequence.")
    print("  At each position, measure the jaw gap and type the value.")
    print("  Press Enter (no value) to skip a point.\n")
    input("  Press Enter when ready to start ...")

    for pulse in TEST_PULSES:
        theoretical_mm = pulse_to_width(pulse) * 1000

        print(f"\n  ► Moving to pulse {pulse}  (theoretical: {theoretical_mm:.1f} mm) ...")
        _move(board, pulse, 1.5)
        time.sleep(1.8)   # wait for servo to fully settle

        # Confirm the servo reached the position
        actual_pulse = _read_position(board)
        temp         = _read_temp(board)

        print(f"    Servo reports: pulse={actual_pulse}  temp={temp}°C")

        measured_mm = prompt_measurement(pulse, theoretical_mm)

        entry = {
            "commanded_pulse":   pulse,
            "actual_pulse":      actual_pulse,
            "theoretical_mm":    round(theoretical_mm, 2),
            "measured_mm":       measured_mm,
            "error_mm":          round(measured_mm - theoretical_mm, 2) if measured_mm is not None else None,
            "temp_c":            temp,
        }
        results.append(entry)

    # Return to open
    print("\n  [INFO] Returning to open position ...")
    open_gripper(board, duration=1.5)
    return results


def print_summary(results: list[dict]) -> None:
    """Print a comparison table and flag any systematic offset."""
    measured = [r for r in results if r["measured_mm"] is not None]

    print("\n" + "═" * 68)
    print("  RESULTS — Theoretical vs Measured")
    print("═" * 68)
    print(f"  {'Pulse':>6}  {'Theory (mm)':>12}  {'Measured (mm)':>14}  {'Error (mm)':>11}")
    print("  " + "─" * 62)

    for r in results:
        m = f"{r['measured_mm']:.1f}" if r["measured_mm"] is not None else "skipped"
        e = f"{r['error_mm']:+.1f}"   if r["error_mm"]   is not None else "   —"
        print(f"  {r['commanded_pulse']:>6}  {r['theoretical_mm']:>12.1f}  {m:>14}  {e:>11}")

    if not measured:
        print("\n  No measurements recorded.")
        return

    errors = [r["error_mm"] for r in measured]
    mean_error = sum(errors) / len(errors)
    max_error  = max(errors, key=abs)

    print("  " + "─" * 62)
    print(f"  Mean error: {mean_error:+.2f} mm")
    print(f"  Max  error: {max_error:+.2f} mm  (at pulse "
          f"{measured[errors.index(max_error)]['commanded_pulse']})")

    print()
    if abs(mean_error) < 1.0:
        print("  [OK]  Model is accurate — linkage constants match your arm.")
    elif abs(mean_error) < 3.0:
        print("  [NOTE] Small systematic offset detected.")
        print(f"         The model consistently reads {mean_error:+.1f} mm vs reality.")
        print("         Fine for most tasks; note this as a calibration offset.")
    else:
        print("  [WARN] Significant offset detected.")
        print(f"         The model is off by ~{mean_error:+.1f} mm on average.")
        print("         Consider adjusting the linkage constants in gripper.py")
        print("         (GRIPPER_HB, GRIPPER_BC) or applying a correction factor.")


def save_results(results: list[dict]) -> None:
    with open(RESULTS_FILE, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n  [OK]  Results saved to {RESULTS_FILE}")


def main() -> None:
    print_position_map()

    board = connect()

    try:
        state = read_gripper_state(board)
        print(f"\n  Current state: pulse={state.pulse}  "
              f"width={state.width_m*1000:.1f}mm  "
              f"temp={state.temp_c}°C  "
              f"voltage={state.voltage_mv}mV")

        results = run_characterization(board)
        print_summary(results)
        save_results(results)

    finally:
        print("\n  [INFO] Opening gripper and closing connection ...")
        open_gripper(board, duration=1.5)
        board.port.close()
        print("  [OK]  Done.")


if __name__ == "__main__":
    main()
