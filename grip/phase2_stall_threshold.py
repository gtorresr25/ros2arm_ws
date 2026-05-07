#!/usr/bin/env python3
"""
phase2_stall_threshold.py — Phase 2: Stall threshold calibration.

Runs 4 passes in one session:
  Pass 0 — empty jaws         (establishes free-movement noise floor)
  Pass 1 — water bottle 35 mm (soft, deformable)
  Pass 2 — case         28 mm (semi-rigid)
  Pass 3 — hard cylinder 27 mm (rigid)

At each step the gripper closes by GRIP_STEP pulses, waits to settle, then
reads back the actual position.  The position error (commanded − actual) is
recorded.  For free movement the error is small; when the jaws contact an
object the servo stalls and the error grows.  The gap between the two is
where STALL_THRESHOLD should live.

The script does NOT use the current STALL_THRESHOLD to stop — it closes until
either a safety ceiling (SAFETY_STOP_ERROR) or PULSE_MAX is reached, so the
full error profile is captured for analysis.

Saves results to phase2_results.json.
Prints a recommended STALL_THRESHOLD at the end.

Usage
-----
    python3 phase2_stall_threshold.py
"""

import json
import os
import sys
import time

from gripper import (
    Board,
    PULSE_OPEN,
    PULSE_MAX,
    GRIP_STEP,
    GRIP_STEP_DELAY,
    GRIP_SETTLE_DELAY,
    STALL_THRESHOLD,
    _move,
    _read_position,
    _read_temp,
    open_gripper,
    width_to_pulse,
)

SERIAL_PORT      = "/dev/ttyUSB0"
BAUD_RATE        = 1000000
RESULTS_FILE     = os.path.join(os.path.dirname(__file__), "phase2_results.json")

# Stop a pass early if error exceeds this — prevents crushing objects.
# Set well above expected stall signal so we still capture the full profile.
SAFETY_STOP_ERROR = 60

# Objects: (label, width_mm, description)
OBJECTS = [
    ("empty",    None, "empty jaws — noise floor baseline"),
    ("bottle",   35,   "water bottle — soft, deformable"),
    ("case",     28,   "case — semi-rigid"),
    ("cylinder", 27,   "hard cylinder — rigid"),
]


def connect() -> Board:
    print(f"\n[INFO] Connecting on {SERIAL_PORT} @ {BAUD_RATE} baud ...")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.")
    return board


def run_pass(board: Board, label: str, width_mm: float | None) -> list[dict]:
    """Close the gripper from pre-open to PULSE_MAX (or safety stop).

    For the empty pass closes from PULSE_OPEN.
    For object passes opens to object width + margin first.
    Records every step regardless of error size.
    """
    if width_mm is not None:
        start_pulse = max(PULSE_OPEN, width_to_pulse(width_mm / 1000) - 30)
        print(f"  Opening to pulse {start_pulse} ({width_mm + (start_pulse - width_to_pulse(width_mm/1000))*0:.0f} mm clearance) ...")
    else:
        start_pulse = PULSE_OPEN

    _move(board, start_pulse, 1.2)
    time.sleep(1.4)

    commanded = start_pulse
    steps = []

    print(f"  {'Step':>4}  {'Cmd':>5}  {'Actual':>7}  {'Error':>6}  {'Temp':>5}")
    print("  " + "─" * 38)

    while commanded + GRIP_STEP <= PULSE_MAX:
        commanded += GRIP_STEP

        _move(board, commanded, GRIP_STEP_DELAY)
        time.sleep(GRIP_STEP_DELAY + GRIP_SETTLE_DELAY)

        actual = _read_position(board)
        temp   = _read_temp(board)

        if actual is None:
            print(f"  [WARN] Read failed at cmd={commanded}, skipping.")
            continue

        error = commanded - actual
        print(f"  {len(steps)+1:>4}  {commanded:>5}  {actual:>7}  {error:>6}  {temp:>4}°C")

        steps.append({
            "label":     label,
            "commanded": commanded,
            "actual":    actual,
            "error":     error,
            "temp_c":    temp,
        })

        if error >= SAFETY_STOP_ERROR:
            print(f"  [STOP] Safety ceiling reached (error {error} ≥ {SAFETY_STOP_ERROR}) — opening.")
            break

    open_gripper(board, duration=1.0)
    return steps


def analyze(all_steps: list[dict]) -> dict:
    """Compute noise floor and contact error for each object pass."""

    # ── Noise floor from empty pass ───────────────────────────────────────────
    # Exclude the last 2 steps approaching PULSE_MAX where linkage resistance
    # naturally increases — this avoids inflating the noise floor.
    empty_steps = [s for s in all_steps if s["label"] == "empty"]
    cutoff_pulse = PULSE_MAX - 2 * GRIP_STEP
    free_errors  = [s["error"] for s in empty_steps if s["commanded"] <= cutoff_pulse]

    noise_floor  = max(free_errors) if free_errors else 0
    noise_mean   = round(sum(free_errors) / len(free_errors), 1) if free_errors else 0

    # ── Contact error per object ──────────────────────────────────────────────
    # First step where error clearly exceeds the noise floor.
    # We use noise_floor + 3 as the detection threshold for this analysis
    # (conservative — just finding where the signal rises above baseline).
    detection_threshold = noise_floor + 3
    contact_errors = {}

    for label, width_mm, _ in OBJECTS[1:]:   # skip empty
        obj_steps = [s for s in all_steps if s["label"] == label]
        contact_pulse = None
        contact_error = None
        for s in obj_steps:
            if s["error"] > detection_threshold:
                contact_pulse = s["commanded"]
                contact_error = s["error"]
                break
        contact_errors[label] = {
            "width_mm":      width_mm,
            "contact_pulse": contact_pulse,
            "contact_error": contact_error,
        }

    # ── Recommended STALL_THRESHOLD ──────────────────────────────────────────
    # Sits halfway between the noise floor and the smallest contact error seen.
    # Rounded to the nearest 5 for a clean constant.
    valid_contact = [v["contact_error"] for v in contact_errors.values() if v["contact_error"] is not None]
    if valid_contact:
        min_contact_error = min(valid_contact)
        midpoint          = (noise_floor + min_contact_error) / 2
        recommended       = int(((midpoint + 2) // 5) * 5)   # round to nearest 5
    else:
        recommended = STALL_THRESHOLD   # fallback: keep current

    return {
        "noise_floor":           noise_floor,
        "noise_mean":            noise_mean,
        "contact_errors":        contact_errors,
        "current_threshold":     STALL_THRESHOLD,
        "recommended_threshold": recommended,
    }


def print_summary(stats: dict) -> None:
    print("\n" + "═" * 58)
    print("  PHASE 2 — RESULTS")
    print("═" * 58)

    print(f"\n  Noise floor (free movement):")
    print(f"    Mean error : {stats['noise_mean']} pulse units")
    print(f"    Max  error : {stats['noise_floor']} pulse units")

    print(f"\n  Contact error per object:")
    print(f"  {'Object':>12}  {'Width':>6}  {'Contact pulse':>14}  {'Error at contact':>17}")
    print("  " + "─" * 56)
    for label, data in stats["contact_errors"].items():
        cp = str(data["contact_pulse"]) if data["contact_pulse"] else "not detected"
        ce = str(data["contact_error"]) if data["contact_error"] else "—"
        print(f"  {label:>12}  {data['width_mm']:>5}mm  {cp:>14}  {ce:>17}")

    print(f"\n  Current  STALL_THRESHOLD : {stats['current_threshold']}")
    print(f"  Recommended STALL_THRESHOLD : {stats['recommended_threshold']}")

    rec = stats["recommended_threshold"]
    cur = stats["current_threshold"]
    print()
    if rec != cur:
        print(f"  → Update STALL_THRESHOLD = {rec} in gripper.py")
    else:
        print("  [OK]  Current threshold matches recommendation.")


def save_results(all_steps: list[dict], stats: dict) -> None:
    output = {"stats": stats, "steps": all_steps}
    with open(RESULTS_FILE, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\n  [OK]  Results saved to {RESULTS_FILE}")


def main() -> None:
    print("═" * 58)
    print("  PHASE 2 — STALL THRESHOLD CALIBRATION")
    print("═" * 58)
    print(f"\n  4 passes:  empty | bottle 35mm | case 28mm | cylinder 27mm")
    print(f"  Safety stop at error ≥ {SAFETY_STOP_ERROR} pulse units")
    print(f"  Pulse range: {PULSE_OPEN} → {PULSE_MAX},  step: {GRIP_STEP}")

    board = connect()
    all_steps = []

    try:
        for label, width_mm, description in OBJECTS:
            print(f"\n{'─'*58}")
            print(f"  Pass: {label.upper()}  —  {description}")
            if width_mm:
                print(f"  Place the {description} between the jaws now.")
            else:
                print(f"  Make sure the jaws are completely empty.")
            print(f"{'─'*58}")
            input("  Press Enter when ready ...")

            steps = run_pass(board, label, width_mm)
            all_steps.extend(steps)
            print(f"  Pass complete — {len(steps)} steps recorded.")

            if label != OBJECTS[-1][0]:
                print("\n  [INFO] Cooling down 5s ...")
                time.sleep(5)

        stats = analyze(all_steps)
        print_summary(stats)
        save_results(all_steps, stats)

    finally:
        print("\n  [INFO] Opening gripper and closing connection ...")
        open_gripper(board, duration=1.5)
        board.port.close()
        print("  [OK]  Done.")


if __name__ == "__main__":
    main()
