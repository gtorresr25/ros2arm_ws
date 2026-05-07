#!/usr/bin/env python3
"""
gripper.py — ArmPi Ultra gripper control module (no ROS required).

Servo 1 is the gripper.  Pulse range 0–1000; 200 = fully open (zero reference).
Higher pulse = more closed.  Physical jaw width is computed from the four-bar
linkage geometry that the manufacturer put in utils.py.

Public API
----------
  pulse_to_width(pulse)                → jaw inner width in metres
  width_to_pulse(width_m)              → pulse (int, clamped to safe range)
  read_gripper_state(board)            → GripperState(position, width_m, temp_c, voltage_mv)
  open_gripper(board, duration=1.0)    → move to fully open
  grip(board, object_width_m, ...)     → close until stall, return GripResult
  release(board, duration=1.0)         → alias for open_gripper

Torque / safety approach
------------------------
The LX-series bus servos expose position, temperature, and voltage — they do
NOT report motor current directly.  Overload protection is therefore built from
two complementary mechanisms:

  1. Position-error stall detection
     The gripper closes in small increments.  After each step the actual
     position is read back.  If (commanded − actual) > STALL_THRESHOLD the
     servo is physically blocked — an object is in the grip.  Closing stops
     immediately.  This is the primary contact signal.

  2. Temperature guard
     The servo temperature is sampled every TEMP_CHECK_EVERY steps.  If the
     reading exceeds TEMP_LIMIT_C the grip attempt is aborted and the gripper
     releases.  This protects against sustained stall load or rapid repeated
     cycles heating up the motor.

Step 1 — Position → width mapping
   pulse_to_width / width_to_pulse use the exact four-bar linkage formulas
   from ArmPi_Ultra_Resources/.../utils.py.  At pulse 200 the jaw is ~61 mm
   wide; at pulse 700 it is ~0 mm (theoretical maximum closing).  Practical
   safe closing is PULSE_MAX = 680.

Step 2 — Torque thresholds (tunable constants near the top of this file)
   STALL_THRESHOLD   primary contact detector (pulse units)
   TEMP_LIMIT_C      thermal fuse (°C)
   PULSE_MAX         hard mechanical stop (pulse)

Step 3 — grip() function
   grip(board, object_width_m) is the callable designed for use in higher-
   level pick-and-place pipelines.  Pass the estimated object width and it
   handles pre-opening, incremental closing, stall detection, and temperature
   checking.  Returns a GripResult you can inspect before proceeding.

Integration note — intelligent_grasp.py
----------------------------------------
This module is intended to replace the bare servo command used in the
ROS2-based intelligent grasp pipeline.  In that pipeline the gripper is closed
with a single blind command:

    set_servo_position(joints_pub, 0.5, ((1, gripper_angle),))

where gripper_angle is computed by set_gripper_size(object_width_m) from
depth-camera measurements.  There is no stall detection or thermal protection
in that path.  grip() in this file provides a drop-in upgrade: it takes the
same object_width_m estimate but closes incrementally with feedback.

Reference file (ROS2 node that drives the full pick-and-place sequence):
    ArmPi_Ultra_Resources/Source Code/ROS2/src/large_models/large_models/intelligent_grasp.py

Relevant call site in that file (transport_thread, ~line 545):
    pick_and_place.pick_without_back(p[0], 80, p[2], p[3], 0.015, ...)
    # p[3] is gripper_angle — the pulse computed from depth-measured object width.
    # Inside pick_without_back (pick_and_place.py line 76):
    #     set_servo_position(joints_pub, 0.5, ((1, gripper_angle),))
    # ← this is where grip() should be called instead.
"""

import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

# ── SDK ───────────────────────────────────────────────────────────────────────
SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)

try:
    from ros_robot_controller_sdk import Board
except ImportError as e:
    raise ImportError(f"Could not import ArmPi SDK: {e}\n  Expected at: {SDK_PATH}") from e

# ── Servo ID ──────────────────────────────────────────────────────────────────
GRIPPER_ID  = 1          # servo 1 drives the gripper jaws
PULSE_OPEN  = 200        # fully open reference (angle_zero in manufacturer code)
PULSE_MAX   = 610        # maximum safe closing pulse — do not exceed
                         # Phase 1 calibration (phase1_results.json) showed jaws
                         # physically touch at pulse 620; 610 leaves a 10-pulse buffer.

# ── Step 1: four-bar linkage geometry (units: metres) ────────────────────────
# These constants match ArmPi_Ultra_Resources/.../app/utils/utils.py exactly.
# NOTE: Phase 1 calibration showed this model overestimates jaw width by
# 6–16 mm across the range.  Use gripper_mapping() for accurate values.
_HB  = 0.014
_BC  = 0.03
_ED  = 0.037
_DC  = 0.022
_EDC = math.radians(180 - 21)
_IH  = 0.02
_IG  = 0.005
_LCD = math.acos((_HB - _IG) / _IH)
_EC  = (_ED**2 + _DC**2 - 2 * _ED * _DC * math.cos(_EDC)) ** 0.5
_ECD = math.acos((_DC**2 + _EC**2 - _ED**2) / (2 * _DC * _EC))
_LC  = math.cos(_LCD + _ECD) * _EC   # constant term in width formula


def _pulse_to_width_theoretical(pulse: int) -> float:
    """Theoretical jaw width in metres from four-bar linkage model.
    Overestimates by 6–16 mm on this arm — use pulse_to_width() instead,
    which uses gripper_mapping() when calibration data is available.
    """
    angle = math.radians((pulse - PULSE_OPEN) / 1000 * 180)
    bj    = math.cos(angle) * _BC
    ke    = _HB + bj - _LC
    return max(0.0, 2 * ke)


def _width_to_pulse_theoretical(width_m: float) -> int:
    """Theoretical pulse from four-bar linkage model (no calibration).
    Use width_to_pulse() instead, which uses gripper_mapping() when available.
    """
    half  = width_m / 2
    a     = max(-1.0, min(1.0, (half - _HB + _LC) / _BC))
    pulse = int(math.degrees(math.acos(a)) / 180 * 1000 + PULSE_OPEN)
    return max(PULSE_OPEN, min(PULSE_MAX, pulse))


# ── Gripper mapping — empirical calibration from phase1_results.json ─────────
# Replaces the theoretical model with linear interpolation between measured
# (pulse, width_mm) pairs.  Falls back to the theoretical model if the
# calibration file is not found.
#
# Source: phase1_results.json  (generated by phase1_characterize.py)
# Format expected: list of dicts with keys "commanded_pulse" and "measured_mm"

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "phase1_results.json")


def gripper_mapping(calibration_file: str = CALIBRATION_FILE) -> tuple[list[int], list[float]]:
    """Load the empirical pulse→width calibration from phase1_results.json.

    Returns two parallel lists (pulses, widths_mm) sorted by pulse, containing
    only the points where a measurement was recorded.  These are used by
    pulse_to_width() and width_to_pulse() for linear interpolation.

    Falls back to an empty result (triggering theoretical fallback) if the
    file is missing or unreadable.

    Reference: phase1_results.json — generated by phase1_characterize.py.
    """
    try:
        with open(calibration_file) as f:
            data = json.load(f)
        points = [
            (int(r["commanded_pulse"]), float(r["measured_mm"]))
            for r in data
            if r.get("measured_mm") is not None
        ]
        points.sort(key=lambda p: p[0])
        pulses = [p[0] for p in points]
        widths = [p[1] for p in points]
        return pulses, widths
    except (FileNotFoundError, KeyError, ValueError):
        return [], []


# Load calibration once at import time.
_CAL_PULSES, _CAL_WIDTHS = gripper_mapping()
_CALIBRATED = len(_CAL_PULSES) >= 2


def pulse_to_width(pulse: int) -> float:
    """Return jaw inner width in metres for a given servo pulse.

    Uses linear interpolation over the empirical calibration points from
    phase1_results.json when available.  Falls back to the theoretical
    four-bar linkage model if calibration data is missing.
    """
    pulse = max(PULSE_OPEN, min(PULSE_MAX, pulse))
    if _CALIBRATED:
        if pulse <= _CAL_PULSES[0]:
            return _CAL_WIDTHS[0] / 1000.0
        if pulse >= _CAL_PULSES[-1]:
            return _CAL_WIDTHS[-1] / 1000.0
        for i in range(len(_CAL_PULSES) - 1):
            p0, p1 = _CAL_PULSES[i], _CAL_PULSES[i + 1]
            if p0 <= pulse <= p1:
                t = (pulse - p0) / (p1 - p0)
                width_mm = _CAL_WIDTHS[i] + t * (_CAL_WIDTHS[i + 1] - _CAL_WIDTHS[i])
                return width_mm / 1000.0
    return _pulse_to_width_theoretical(pulse)


def width_to_pulse(width_m: float) -> int:
    """Return the servo pulse needed to achieve a target jaw width (metres).

    Uses linear interpolation over the empirical calibration points from
    phase1_results.json when available.  Falls back to the theoretical
    four-bar linkage model if calibration data is missing.

    Result is clamped to [PULSE_OPEN, PULSE_MAX].
    """
    width_mm = width_m * 1000.0
    if _CALIBRATED:
        if width_mm >= _CAL_WIDTHS[0]:
            return _CAL_PULSES[0]
        if width_mm <= _CAL_WIDTHS[-1]:
            return _CAL_PULSES[-1]
        for i in range(len(_CAL_WIDTHS) - 1):
            w0, w1 = _CAL_WIDTHS[i], _CAL_WIDTHS[i + 1]
            if w1 <= width_mm <= w0:   # widths decrease as pulse increases
                t = (w0 - width_mm) / (w0 - w1)
                pulse = int(_CAL_PULSES[i] + t * (_CAL_PULSES[i + 1] - _CAL_PULSES[i]))
                return max(PULSE_OPEN, min(PULSE_MAX, pulse))
    return _width_to_pulse_theoretical(width_m)


# ── Step 2: safety thresholds ─────────────────────────────────────────────────
# Calibrated against phase2_results.json.  Three defined tiers:
#
#   STALL_THRESHOLD_FRAGILE = 30   fragile or compressible objects (fruit, foam, thin shells)
#                                  +12 units above startup spike (18) — safe margin
#                                  rigid compression ≈ 3 mm, soft ≈ 18 mm
#
#   STALL_THRESHOLD         = 45   NOMINAL — use this for all standard objects
#                                  +27 units above startup spike — very robust
#                                  rigid compression ≈ 4 mm, soft ≈ 21 mm
#
#   STALL_THRESHOLD_MAX     = 60   upper limit — rigid/semi-rigid objects only
#                                  soft/deformable objects (e.g. empty bottle) are NOT
#                                  detected at this threshold within the pulse range.
#                                  Do not use for unknown or soft objects.
#
# Noise floor reference (phase2_results.json):
#   sustained free-movement max : 16 pulse units
#   startup transient spike     : 18 pulse units  (first ~3 steps from rest)

STALL_THRESHOLD_FRAGILE = 30
STALL_THRESHOLD         = 40   # nominal — default used by grip()
STALL_THRESHOLD_MAX     = 60

TEMP_LIMIT_C      = 60    # °C — abort and release if servo reaches this temperature
                           # LX-series servos are rated to ~70 °C; 60 °C gives a safe margin

TEMP_CHECK_EVERY  = 5     # check temperature every N steps (each step is ~50 ms)

GRIP_STEP         = 10    # pulse units per closing increment (≈ 1.8° per step)
                           # Smaller = finer contact detection; larger = faster close

GRIP_STEP_DELAY   = 0.05  # seconds to execute each step (motion duration sent to servo)
GRIP_SETTLE_DELAY = 0.04  # extra seconds to wait for servo to settle before reading back

PRE_OPEN_MARGIN   = 0.012 # metres — open this much wider than the object estimate before closing
                           # 12 mm gives clearance for positioning errors

VOLTAGE_LOW_MV    = 6500  # mV — warn if battery below this during grip attempt


# ── Data types ────────────────────────────────────────────────────────────────
@dataclass
class GripperState:
    """Snapshot of the gripper servo at a point in time."""
    pulse:      int           # current servo pulse position
    width_m:    float         # computed jaw inner width in metres
    temp_c:     Optional[int] # servo temperature in °C (None if read failed)
    voltage_mv: Optional[int] # servo voltage in mV   (None if read failed)


@dataclass
class GripResult:
    """Outcome of a grip() call."""
    success:     bool   # True = object contacted and held; False = something went wrong
    final_pulse: int    # servo pulse at the moment closing stopped
    final_width_m: float# jaw width at that pulse (metres)
    stop_reason: str    # "stall"           — normal contact detection (good)
                        # "temp_limit"      — servo overheated; grip released
                        # "max_pulse"       — reached PULSE_MAX without contact (object may be
                        #                     smaller than estimated or missing)
                        # "low_voltage"     — battery too low to grip safely
                        # "read_error"      — servo not responding

    def __str__(self) -> str:
        status = "GRIPPED" if self.success else "FAILED"
        w_mm = self.final_width_m * 1000
        return (f"GripResult({status}  pulse={self.final_pulse}"
                f"  jaw={w_mm:.1f}mm  reason={self.stop_reason})")


# ── Internal helpers ──────────────────────────────────────────────────────────
def _read_position(board: Board) -> Optional[int]:
    result = board.bus_servo_read_position(GRIPPER_ID)
    return result[0] if result is not None else None


def _read_temp(board: Board) -> Optional[int]:
    result = board.bus_servo_read_temp(GRIPPER_ID)
    return result[0] if result is not None else None


def _read_voltage(board: Board) -> Optional[int]:
    result = board.bus_servo_read_vin(GRIPPER_ID)
    return result[0] if result is not None else None


def _move(board: Board, pulse: int, duration: float) -> None:
    """Send a single position command to the gripper servo."""
    board.bus_servo_set_position(duration, [[GRIPPER_ID, pulse]])


# ── Public API ────────────────────────────────────────────────────────────────
def read_gripper_state(board: Board) -> GripperState:
    """Read position, temperature, and voltage from the gripper servo."""
    pulse   = _read_position(board) or PULSE_OPEN
    width_m = pulse_to_width(pulse)
    temp_c  = _read_temp(board)
    voltage = _read_voltage(board)
    return GripperState(pulse=pulse, width_m=width_m, temp_c=temp_c, voltage_mv=voltage)


def open_gripper(board: Board, duration: float = 1.0) -> None:
    """Move the gripper to the fully open position."""
    _move(board, PULSE_OPEN, duration)
    time.sleep(duration + 0.1)


def release(board: Board, duration: float = 1.0) -> None:
    """Alias for open_gripper — use this after placing an object."""
    open_gripper(board, duration)


def grip(
    board:            Board,
    object_width_m:   float,
    *,
    pre_open_margin:  float = PRE_OPEN_MARGIN,
    stall_threshold:  int   = STALL_THRESHOLD,
    temp_limit_c:     int   = TEMP_LIMIT_C,
    step:             int   = GRIP_STEP,
    step_delay:       float = GRIP_STEP_DELAY,
    settle_delay:     float = GRIP_SETTLE_DELAY,
    verbose:          bool  = True,
) -> GripResult:
    """Close the gripper onto an object of the given estimated width.

    Parameters
    ----------
    board            : connected Board instance
    object_width_m   : estimated object width in metres (e.g. 0.04 for a 40 mm cube)
    pre_open_margin  : open this much wider than the estimate before closing (metres)
    stall_threshold  : position-error threshold in pulse units for contact detection
    temp_limit_c     : abort temperature in °C
    step             : pulse increment per closing step
    step_delay       : duration (s) sent to the servo for each step
    settle_delay     : extra wait (s) after each step before reading position
    verbose          : print progress to stdout

    Returns
    -------
    GripResult — inspect .success and .stop_reason before continuing.

    Typical usage
    -------------
        result = grip(board, object_width_m=0.04)
        if result.success:
            # carry object, then release
            release(board)
        else:
            print("Grip failed:", result)
    """
    def log(msg: str) -> None:
        if verbose:
            print(msg)

    # ── Pre-flight: check voltage ─────────────────────────────────────────────
    voltage = _read_voltage(board)
    if voltage is not None and voltage < VOLTAGE_LOW_MV:
        log(f"[WARN] Battery low: {voltage} mV  (threshold {VOLTAGE_LOW_MV} mV)")
        return GripResult(
            success=False,
            final_pulse=PULSE_OPEN,
            final_width_m=pulse_to_width(PULSE_OPEN),
            stop_reason="low_voltage",
        )

    # ── Step 1: open to pre-grasp width ──────────────────────────────────────
    pre_open_width  = object_width_m + pre_open_margin
    pre_open_pulse  = width_to_pulse(pre_open_width)
    pre_open_pulse  = max(PULSE_OPEN, min(PULSE_MAX, pre_open_pulse))

    log(f"[GRIP] object estimate: {object_width_m*1000:.1f} mm"
        f"  →  opening to {pulse_to_width(pre_open_pulse)*1000:.1f} mm"
        f"  (pulse {pre_open_pulse})")

    _move(board, pre_open_pulse, 0.8)
    time.sleep(0.9)

    # Confirm the servo reached the open position
    actual = _read_position(board)
    if actual is None:
        return GripResult(
            success=False,
            final_pulse=pre_open_pulse,
            final_width_m=pulse_to_width(pre_open_pulse),
            stop_reason="read_error",
        )
    log(f"[GRIP] pre-open actual pulse: {actual}")

    # ── Step 2 & 3: incremental close with stall and temperature monitoring ───
    commanded = pre_open_pulse
    stop_reason = "max_pulse"
    step_count  = 0

    while commanded + step <= PULSE_MAX:
        commanded += step
        step_count += 1

        # Send the next closing increment
        _move(board, commanded, step_delay)
        time.sleep(step_delay + settle_delay)

        # Read actual position
        actual = _read_position(board)
        if actual is None:
            log(f"[WARN] Position read failed at step {step_count}; aborting.")
            stop_reason = "read_error"
            break

        error = commanded - actual   # positive means servo is lagging (stalled)
        log(f"  step {step_count:3d}  cmd={commanded:4d}  actual={actual:4d}"
            f"  error={error:3d}  jaw≈{pulse_to_width(actual)*1000:.1f}mm")

        # ── Stall detection (primary torque proxy) ────────────────────────────
        if error > stall_threshold:
            log(f"[GRIP] Contact detected at jaw ≈ {pulse_to_width(actual)*1000:.1f} mm"
                f"  (error {error} > threshold {stall_threshold})")
            stop_reason = "stall"
            break

        # ── Temperature check ─────────────────────────────────────────────────
        if step_count % TEMP_CHECK_EVERY == 0:
            temp = _read_temp(board)
            if temp is not None:
                log(f"  [TEMP] servo temp: {temp} °C")
                if temp >= temp_limit_c:
                    log(f"[WARN] Temp limit reached ({temp} °C ≥ {temp_limit_c} °C) — releasing.")
                    open_gripper(board, duration=0.5)
                    return GripResult(
                        success=False,
                        final_pulse=actual,
                        final_width_m=pulse_to_width(actual),
                        stop_reason="temp_limit",
                    )

    # Use the last confirmed actual position as the result
    final_pulse = actual if actual is not None else commanded
    result = GripResult(
        success=(stop_reason == "stall"),
        final_pulse=final_pulse,
        final_width_m=pulse_to_width(final_pulse),
        stop_reason=stop_reason,
    )
    log(f"[GRIP] {result}")
    return result


# ── smart_grip — runtime monitor with operation logging ──────────────────────
#
# smart_grip() is the recommended entry point for all production grip attempts.
# It wraps grip() with:
#   - Two operating modes (estimated width vs unknown object)
#   - Contact window verification against the estimate
#   - Outcome classification (SECURE / MARGINAL_LARGE / MARGINAL_SMALL / FAILED)
#   - One log record per attempt written to GRIP_LOG_FILE
#
# TODO — slip monitoring hook (Phase 4):
#   After gripping and during arm transport, periodically call _read_position()
#   and compare against the grip pulse recorded in SmartGripResult.grip_pulse.
#   If the actual position drifts toward the commanded pulse by > SLIP_THRESHOLD
#   (12 pulse units), the object has slipped.  Trigger a re-grip or abort.
#   A monitor_slip(board, result, duration) helper should be added here that
#   the pick-and-place pipeline calls between grip and place.

GRIP_LOG_FILE = os.path.join(os.path.dirname(__file__), "grip_log.json")

# Contact window bounds (Mode 1 only)
EARLY_CONTACT_MM = 7.0    # object larger than estimated if contact > 7 mm early
LATE_CONTACT_MM  = 10.0   # object smaller than estimated if contact > 10 mm late
                           # asymmetric: late is riskier (more force applied before stop)

# Mode 2 defaults — safer, slower, lower threshold
UNKNOWN_GRIP_STEP    = 5   # half normal step size
UNKNOWN_GRIP_DELAY   = 0.12
UNKNOWN_GRIP_SETTLE  = 0.06


@dataclass
class SmartGripResult:
    """Outcome of a smart_grip() call."""
    # ── Core grip result ──────────────────────────────────────────────────────
    success:            bool         # True = object contacted and held
    grip_pulse:         int          # actual servo pulse at contact
    jaw_at_contact_mm:  float        # jaw inner width at contact (mm)
    stop_reason:        str          # from grip() — "stall" | "max_pulse" | etc.

    # ── Monitor outcome ───────────────────────────────────────────────────────
    outcome:            str          # "SECURE" | "MARGINAL_LARGE" | "MARGINAL_SMALL" | "FAILED"
    mode:               str          # "estimated" | "unknown"
    estimate_source:    str          # "depth_camera" | "manual" | "unknown"
    estimated_width_mm: Optional[float]  # None if mode == "unknown"
    contact_offset_mm:  Optional[float]  # actual_jaw - estimated_width
                                         # positive = object larger than expected
                                         # negative = object smaller than expected
                                         # None if mode == "unknown" or no contact
    notes:              list[str]    # human-readable flags logged per attempt
    temp_c:             Optional[int]

    def __str__(self) -> str:
        est = f"{self.estimated_width_mm:.1f}mm" if self.estimated_width_mm else "unknown"
        off = f"{self.contact_offset_mm:+.1f}mm" if self.contact_offset_mm is not None else "n/a"
        return (f"SmartGripResult({self.outcome}"
                f"  jaw={self.jaw_at_contact_mm:.1f}mm"
                f"  est={est}  offset={off}"
                f"  mode={self.mode})")


def _append_log(record: dict) -> None:
    """Append one grip record to GRIP_LOG_FILE (creates file if absent)."""
    log = []
    if os.path.exists(GRIP_LOG_FILE):
        try:
            with open(GRIP_LOG_FILE) as f:
                log = json.load(f)
        except (json.JSONDecodeError, ValueError):
            log = []
    log.append(record)
    with open(GRIP_LOG_FILE, "w") as f:
        json.dump(log, f, indent=2)


def smart_grip(
    board:              Board,
    estimated_width_m:  Optional[float] = None,
    estimate_source:    str             = "unknown",
    *,
    early_contact_mm:   float           = EARLY_CONTACT_MM,
    late_contact_mm:    float           = LATE_CONTACT_MM,
    log_file:           str             = GRIP_LOG_FILE,
    verbose:            bool            = True,
) -> SmartGripResult:
    """Runtime grip monitor — recommended entry point for all grip attempts.

    Parameters
    ----------
    board              : connected Board instance
    estimated_width_m  : estimated object width in metres, or None for unknown
    estimate_source    : where the estimate came from — "depth_camera" | "manual" | "unknown"
    early_contact_mm   : flag MARGINAL_LARGE if contact > this many mm before estimate
    late_contact_mm    : flag MARGINAL_SMALL if contact > this many mm after estimate
    log_file           : path to the operation log (default: grip_log.json)
    verbose            : print progress to stdout

    Returns
    -------
    SmartGripResult — check .outcome before continuing.
        "SECURE"         → gripped within expected bounds, proceed normally
        "MARGINAL_LARGE" → object larger than estimated, gripped but flag it
        "MARGINAL_SMALL" → object smaller than estimated, gripped but flag it
        "FAILED"         → no contact detected, abort

    Operating modes
    ---------------
    Mode 1 — estimated_width_m provided:
        Opens to estimate + margin, closes at STALL_THRESHOLD (40).
        Verifies contact falls within ±early/late_contact_mm of estimate.

    Mode 2 — estimated_width_m is None (unknown object):
        Opens fully, closes at STALL_THRESHOLD_FRAGILE (30) with smaller
        steps — slower, gentler, stops sooner.  Any contact = SECURE.
    """
    import datetime
    timestamp = datetime.datetime.now().isoformat(timespec="seconds")
    notes: list[str] = []

    def log(msg: str) -> None:
        if verbose:
            print(msg)

    # ── Mode selection ────────────────────────────────────────────────────────
    if estimated_width_m is None:
        mode = "unknown"
        log(f"[SMART] Mode 2 — unknown object  "
            f"(threshold={STALL_THRESHOLD_FRAGILE}, step={UNKNOWN_GRIP_STEP})")
        notes.append("unknown object — conservative mode")

        result = grip(
            board,
            object_width_m  = pulse_to_width(PULSE_OPEN),   # open fully
            pre_open_margin = 0.0,
            stall_threshold = STALL_THRESHOLD_FRAGILE,
            step            = UNKNOWN_GRIP_STEP,
            step_delay      = UNKNOWN_GRIP_DELAY,
            settle_delay    = UNKNOWN_GRIP_SETTLE,
            verbose         = verbose,
        )
        threshold_used = STALL_THRESHOLD_FRAGILE

    else:
        mode = "estimated"
        est_mm = estimated_width_m * 1000
        log(f"[SMART] Mode 1 — estimate: {est_mm:.1f} mm  source: {estimate_source}"
            f"  threshold: {STALL_THRESHOLD}")

        result = grip(
            board,
            object_width_m  = estimated_width_m,
            stall_threshold = STALL_THRESHOLD,
            verbose         = verbose,
        )
        threshold_used = STALL_THRESHOLD

    # ── Read temperature at end of grip ───────────────────────────────────────
    temp_c = _read_temp(board)

    # ── Outcome classification ────────────────────────────────────────────────
    jaw_mm          = result.final_width_m * 1000
    contact_offset  = None

    if not result.success:
        outcome = "FAILED"
        notes.append(f"no contact — stop_reason: {result.stop_reason}")

    elif mode == "unknown":
        outcome = "SECURE"
        notes.append(f"unknown object gripped at jaw={jaw_mm:.1f}mm")

    else:
        # offset: positive = object larger, negative = object smaller
        contact_offset = jaw_mm - (estimated_width_m * 1000)

        if contact_offset > early_contact_mm:
            outcome = "MARGINAL_LARGE"
            notes.append(
                f"object larger than estimated by {contact_offset:.1f}mm "
                f"(threshold: +{early_contact_mm}mm) — check estimate source"
            )
            log(f"[SMART] MARGINAL_LARGE — contacted {contact_offset:.1f}mm early")

        elif (-contact_offset) > late_contact_mm:
            outcome = "MARGINAL_SMALL"
            notes.append(
                f"object smaller than estimated by {-contact_offset:.1f}mm "
                f"(threshold: -{late_contact_mm}mm) — check estimate source"
            )
            log(f"[SMART] MARGINAL_SMALL — contacted {-contact_offset:.1f}mm late")

        else:
            outcome = "SECURE"

    log(f"[SMART] outcome={outcome}  jaw={jaw_mm:.1f}mm  offset={contact_offset}")

    # ── Build result ──────────────────────────────────────────────────────────
    smart_result = SmartGripResult(
        success            = result.success,
        grip_pulse         = result.final_pulse,
        jaw_at_contact_mm  = round(jaw_mm, 1),
        stop_reason        = result.stop_reason,
        outcome            = outcome,
        mode               = mode,
        estimate_source    = estimate_source,
        estimated_width_mm = round(estimated_width_m * 1000, 1) if estimated_width_m else None,
        contact_offset_mm  = round(contact_offset, 1) if contact_offset is not None else None,
        notes              = notes,
        temp_c             = temp_c,
    )

    # ── Write operation log ───────────────────────────────────────────────────
    _append_log({
        "timestamp":          timestamp,
        "mode":               mode,
        "estimate_source":    estimate_source,
        "estimated_width_mm": smart_result.estimated_width_mm,
        "actual_jaw_mm":      smart_result.jaw_at_contact_mm,
        "contact_offset_mm":  smart_result.contact_offset_mm,
        "threshold":          threshold_used,
        "outcome":            outcome,
        "stop_reason":        result.stop_reason,
        "grip_pulse":         result.final_pulse,
        "temp_c":             temp_c,
        "notes":              notes,
    })

    return smart_result


# ── Calibration helper ────────────────────────────────────────────────────────
def print_position_map() -> None:
    """Print pulse → jaw width for reference.  No hardware required."""
    print(f"\n{'Pulse':>6}  {'Width (mm)':>10}  {'Notes'}")
    print("-" * 40)
    notes = {
        PULSE_OPEN: "fully open (angle_zero)",
        400: "",
        500: "centre of range",
        570: "small object close (~25 mm)",
        PULSE_MAX: f"PULSE_MAX (hard limit)",
    }
    for p in list(range(PULSE_OPEN, PULSE_MAX + 1, 20)) + list(notes.keys()):
        p = max(PULSE_OPEN, min(PULSE_MAX, p))
        w = pulse_to_width(p) * 1000
        note = notes.get(p, "")
        print(f"  {p:4d}     {w:7.1f} mm   {note}")


# ── Standalone demo ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    import os

    print_position_map()

    SERIAL_PORT = "/dev/ttyUSB0"
    BAUD_RATE   = 1000000

    print("\n[INFO] Connecting to arm ...")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.")

    # Read current state
    state = read_gripper_state(board)
    print(f"\nCurrent gripper state:")
    print(f"  Pulse:    {state.pulse}")
    print(f"  Width:    {state.width_m*1000:.1f} mm")
    print(f"  Temp:     {state.temp_c} °C")
    print(f"  Voltage:  {state.voltage_mv} mV  ({(state.voltage_mv or 0)/1000:.2f} V)")

    ans = input("\nRun grip test on a ~40 mm object? (place an object in the jaws first) [y/N] ").strip().lower()
    if ans == "y":
        print("\n[INFO] Opening gripper ...")
        open_gripper(board)

        result = grip(board, object_width_m=0.040)
        print(f"\nResult: {result}")

        if result.success:
            input("\nGripper is holding object.  Press Enter to release ...")
            release(board)
        else:
            print("[INFO] Grip did not detect contact — gripper left in current state.")
            open_gripper(board)
    else:
        print("[INFO] Test skipped.")

    board.port.close()
    print("[OK]  Done.")
