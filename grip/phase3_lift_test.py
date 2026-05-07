#!/usr/bin/env python3
"""
phase3_lift_test.py — Phase 3: Grip security / lift test.

Closes the gripper slowly onto an object at the nominal STALL_THRESHOLD (40),
lifts the arm slightly, then checks whether the object was held throughout.

Slow close
----------
LIFT_GRIP_STEP = 5 (half the normal 10) with longer delays — closing is
visually trackable and gentle.

Lift
----
Reads the current shoulder position (servo 5) and raises it by LIFT_DELTA
pulses — a small conservative arc, enough to lift the gripper tip a few cm.

Slip detection
--------------
After gripping, monitors the gripper's actual position during the lift.
If it drifts toward the commanded pulse by more than SLIP_THRESHOLD, the
object has slipped or dropped.

Usage
-----
    python3 phase3_lift_test.py

Saves results to phase3_results.json (appends each run — run as many objects
as needed before quitting with q).
"""

import json
import os
import time

from gripper import (
    Board,
    PULSE_OPEN,
    PULSE_MAX,
    STALL_THRESHOLD,
    _move,
    _read_position,
    _read_temp,
    open_gripper,
    pulse_to_width,
    width_to_pulse,
)

SERIAL_PORT  = "/dev/ttyUSB0"
BAUD_RATE    = 1000000
RESULTS_FILE = os.path.join(os.path.dirname(__file__), "phase3_results.json")

# ── Slow close ────────────────────────────────────────────────────────────────
LIFT_GRIP_STEP   = 5     # pulse units per step (normal = 10)
LIFT_GRIP_DELAY  = 0.12  # seconds per step motion
LIFT_GRIP_SETTLE = 0.06  # extra settle before reading

# ── Lift ──────────────────────────────────────────────────────────────────────
SHOULDER_SERVO = 5
LIFT_DELTA     = 80     # pulse units to raise shoulder
LIFT_DURATION  = 2.0    # seconds for the lift movement
HOLD_DURATION  = 3.0    # seconds to hold at top before lowering

# ── Slip detection ─────────────────────────────────────────────────────────────
SLIP_THRESHOLD = 12     # pulse units of drift toward commanded = object slipped


def connect() -> Board:
    print(f"\n[INFO] Connecting on {SERIAL_PORT} @ {BAUD_RATE} baud ...")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.")
    return board


def read_shoulder(board: Board) -> int | None:
    result = board.bus_servo_read_position(SHOULDER_SERVO)
    return result[0] if result is not None else None


def slow_grip(board: Board, object_width_m: float) -> tuple[bool, int, list[dict]]:
    """Close slowly until stall or PULSE_MAX.

    Returns (contacted, grip_pulse, steps).
    """
    pre_open_pulse = max(PULSE_OPEN, width_to_pulse(object_width_m) - 30)
    _move(board, pre_open_pulse, 1.2)
    time.sleep(1.4)

    commanded = pre_open_pulse
    steps     = []
    contacted = False

    print(f"\n  Closing slowly — step={LIFT_GRIP_STEP}, threshold={STALL_THRESHOLD} ...")
    print(f"  {'Step':>4}  {'Cmd':>5}  {'Actual':>7}  {'Error':>6}  {'Jaw mm':>8}")
    print("  " + "─" * 42)

    while commanded + LIFT_GRIP_STEP <= PULSE_MAX:
        commanded += LIFT_GRIP_STEP

        _move(board, commanded, LIFT_GRIP_DELAY)
        time.sleep(LIFT_GRIP_DELAY + LIFT_GRIP_SETTLE)

        actual = _read_position(board)
        temp   = _read_temp(board)
        if actual is None:
            print(f"  [WARN] Read failed at cmd={commanded}")
            continue

        error  = commanded - actual
        jaw_mm = pulse_to_width(actual) * 1000
        print(f"  {len(steps)+1:>4}  {commanded:>5}  {actual:>7}  {error:>6}  {jaw_mm:>7.1f}mm")

        steps.append({
            "commanded": commanded,
            "actual":    actual,
            "error":     error,
            "jaw_mm":    round(jaw_mm, 1),
            "temp_c":    temp,
        })

        if error >= STALL_THRESHOLD:
            print(f"\n  [GRIP] Contact — error {error} >= {STALL_THRESHOLD}")
            print(f"         Jaw at contact: {jaw_mm:.1f} mm")
            contacted = True
            break

    grip_pulse = actual if actual is not None else commanded
    return contacted, grip_pulse, steps


def lift_and_check(board: Board, grip_pulse: int) -> dict:
    """Raise shoulder, hold, monitor gripper, lower back."""
    shoulder_start = read_shoulder(board)
    if shoulder_start is None:
        print("  [WARN] Could not read shoulder — skipping lift.")
        return {"lifted": False, "slip_detected": False, "max_drift": None}

    shoulder_target = min(850, shoulder_start + LIFT_DELTA)

    print(f"\n  Lifting — servo 5:  {shoulder_start} → {shoulder_target}  over {LIFT_DURATION}s ...")
    board.bus_servo_set_position(LIFT_DURATION, [[SHOULDER_SERVO, shoulder_target]])
    time.sleep(LIFT_DURATION + 0.3)

    print(f"  Holding {HOLD_DURATION}s — monitoring gripper ...")
    readings = []
    t_start  = time.time()
    while time.time() - t_start < HOLD_DURATION:
        actual = _read_position(board)
        if actual is not None:
            # Negative drift = gripper moved toward commanded = object slipped
            drift = actual - grip_pulse
            slip  = (-drift) > SLIP_THRESHOLD
            readings.append({"actual": actual, "drift": drift})
            flag  = "  *** SLIP ***" if slip else ""
            print(f"    actual={actual}  drift={drift:+d}{flag}")
        time.sleep(0.4)

    print(f"\n  Lowering — servo 5 → {shoulder_start} ...")
    board.bus_servo_set_position(LIFT_DURATION, [[SHOULDER_SERVO, shoulder_start]])
    time.sleep(LIFT_DURATION + 0.3)

    drifts        = [r["drift"] for r in readings]
    max_slip_drift = min(drifts) if drifts else 0
    slip_detected  = (-max_slip_drift) > SLIP_THRESHOLD

    return {
        "lifted":        True,
        "slip_detected": slip_detected,
        "max_drift":     max_slip_drift,
        "readings":      readings,
    }


def load_results() -> list:
    if os.path.exists(RESULTS_FILE):
        with open(RESULTS_FILE) as f:
            return json.load(f)
    return []


def save_results(results: list) -> None:
    with open(RESULTS_FILE, "w") as f:
        json.dump(results, f, indent=2)
    print(f"  [OK]  Saved to {RESULTS_FILE}")


def print_summary(results: list) -> None:
    print("\n" + "═" * 56)
    print("  SESSION SUMMARY")
    print("═" * 56)
    print(f"  {'Object':>12}  {'Width':>6}  {'Jaw at grip':>12}  {'Held':>5}")
    print("  " + "─" * 44)
    for r in results:
        held = "NO  ← SLIPPED" if r["lift"]["slip_detected"] else "YES"
        print(f"  {r['object']:>12}  {r['object_width_mm']:>5}mm  "
              f"{r['jaw_at_grip_mm']:>9.1f}mm  {held}")


def main() -> None:
    board   = connect()
    session = load_results()

    try:
        while True:
            print("\n" + "─" * 56)
            obj_name = input("  Object name (or q to quit): ").strip()
            if obj_name.lower() == "q":
                break
            try:
                obj_width_mm = float(input("  Object width in mm: ").strip())
            except ValueError:
                print("  Invalid width.")
                continue

            print(f"\n  Place the {obj_name} between the jaws.")
            input("  Press Enter when ready ...")

            open_gripper(board, duration=1.0)
            contacted, grip_pulse, grip_steps = slow_grip(board, obj_width_mm / 1000)

            if not contacted:
                print("\n  [WARN] No contact — object may be missing or too small.")
                open_gripper(board)
                continue

            jaw_at_grip = pulse_to_width(grip_pulse) * 1000
            print(f"\n  Gripped: pulse={grip_pulse}  jaw={jaw_at_grip:.1f}mm")
            input("  Press Enter to lift ...")

            lift_result = lift_and_check(board, grip_pulse)

            if lift_result["slip_detected"]:
                print("\n  [RESULT] SLIPPED — not held securely at threshold 40.")
            else:
                print("\n  [RESULT] HELD — secure grip throughout lift.")

            input("  Press Enter to release ...")
            open_gripper(board)

            session.append({
                "object":          obj_name,
                "object_width_mm": obj_width_mm,
                "threshold":       STALL_THRESHOLD,
                "contacted":       contacted,
                "grip_pulse":      grip_pulse,
                "jaw_at_grip_mm":  round(jaw_at_grip, 1),
                "lift":            lift_result,
                "grip_steps":      grip_steps,
            })

            print_summary(session)

    finally:
        save_results(session)
        print("\n  [INFO] Opening gripper and closing connection ...")
        open_gripper(board, duration=1.5)
        board.port.close()
        print("  [OK]  Done.")


if __name__ == "__main__":
    main()
