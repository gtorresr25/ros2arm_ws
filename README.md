# ArmPi Ultra — Development Tools

Standalone scripts for the ArmPi Ultra robotic arm.
No ROS required unless noted.  All scripts connect via `/dev/ttyUSB0` at 1 000 000 baud.

---

## Arm Home

Tools for recording and replaying named arm poses.

### Files
| File | Purpose |
|---|---|
| `arm_homes.py` | CLI — record, recall, and list home positions |
| `home_positions.json` | Saved poses (auto-updated by arm_homes.py) |

### Usage
```bash
# Release torque, position arm by hand, save pose
python3 arm_homes.py release

# Read current servo positions and save as a named pose
python3 arm_homes.py record <name>

# Move arm to a saved pose (optional duration in seconds)
python3 arm_homes.py goto <name> [duration]

# List all saved poses
python3 arm_homes.py list
```

### Saved poses
| Name | S1 | S2 | S3 | S4 | S5 | S6 | Notes |
|---|---|---|---|---|---|---|---|
| home1 | 230 | 498 | 111 | 824 | 603 | 504 | |
| home2 | 230 | 499 | 502 | 496 | 510 | 504 | Straight up |
| home3 | 230 | 498 | 260 | 829 | 716 | 504 | |
| home4 | 230 | 498 | 82  | 749 | 597 | 504 | |

### Servo layout
| ID | Joint |
|---|---|
| 1 | Gripper |
| 2 | Wrist rotation |
| 3 | Wrist pitch |
| 4 | Elbow |
| 5 | Shoulder |
| 6 | Base rotation |

### SDK quirk
`bus_servo_enable_torque(id, True)` = UNLOAD (releases, arm goes limp)
`bus_servo_enable_torque(id, False)` = LOAD (engages, arm holds)
The argument name is inverted relative to its effect.

---

## Gripper Calibration

Tools for characterizing the gripper, mapping pulse to physical jaw width,
and tuning safe grip parameters.  Servo 1 is the gripper.
Pulse range: 200 (fully open) → 610 (safe maximum).

All gripper files live in the `grip/` subfolder.  Run scripts from inside it:
```bash
cd grip/
python3 phase1_characterize.py
```

### Files (`grip/`)
| File | Purpose |
|---|---|
| `gripper.py` | Core module — all gripper functions live here |
| `phase1_characterize.py` | Phase 1 — maps pulse to physical jaw width |
| `phase1_results.json` | Phase 1 measurements (input to gripper_mapping) |
| `phase2_stall_threshold.py` | Phase 2 — stall threshold calibration |
| `phase2_results.json` | Phase 2 measurements (noise floor + contact errors) |
| `phase3_lift_test.py` | Phase 3 — grip security / lift test |
| `phase3_results.json` | Phase 3 results (appended each session) |
| `test_smart_grip.py` | Interactive test for smart_grip() |
| `grip_log.json` | Operation log — one record per smart_grip() call |
| `test_arm_connection.py` | Minimal connection and servo sanity check |

### gripper.py — public API
```python
from gripper import (
    gripper_mapping,      # load empirical pulse→width calibration table
    pulse_to_width,       # pulse (int) → jaw width (metres)
    width_to_pulse,       # jaw width (metres) → pulse (int)
    read_gripper_state,   # → GripperState(pulse, width_m, temp_c, voltage_mv)
    open_gripper,         # move to fully open
    grip,                 # low-level: close until stall, return GripResult
    release,              # alias for open_gripper
    smart_grip,           # recommended entry point — monitor + logging
)
```

### smart_grip() — recommended entry point
Use this for all production grip attempts.  Wraps `grip()` with contact
verification, outcome classification, and automatic operation logging.

```python
from gripper import smart_grip, STALL_THRESHOLD_FRAGILE

# Mode 1 — width estimate available (depth camera, manual measurement, etc.)
result = smart_grip(
    board,
    estimated_width_m = 0.035,      # metres
    estimate_source   = "manual",   # "depth_camera" | "manual" | "unknown"
)

# Mode 2 — unknown object (opens fully, slower, lower threshold)
result = smart_grip(board)

# result.outcome  → "SECURE" | "MARGINAL_LARGE" | "MARGINAL_SMALL" | "FAILED"
# result.jaw_at_contact_mm  → actual jaw width at contact
# result.contact_offset_mm  → actual - estimated (+ = larger, - = smaller)
# result.notes              → list of human-readable flags
```

**Outcome meanings:**
| Outcome | Meaning | Action |
|---|---|---|
| `SECURE` | Contact within expected bounds | Proceed |
| `MARGINAL_LARGE` | Object >7 mm larger than estimated | Proceed, check estimate source |
| `MARGINAL_SMALL` | Object >10 mm smaller than estimated | Proceed with caution |
| `FAILED` | No contact detected | Abort, do not lift |

Every call appends one record to `grip_log.json` regardless of outcome.

### grip_log.json — operation log
One record per grip attempt.  Use this to find patterns over time.

```json
{
  "timestamp":          "2026-04-07T14:32:01",
  "mode":               "estimated",
  "estimate_source":    "depth_camera",
  "estimated_width_mm": 35.0,
  "actual_jaw_mm":      33.2,
  "contact_offset_mm":  -1.8,
  "threshold":          40,
  "outcome":            "SECURE",
  "stop_reason":        "stall",
  "grip_pulse":         528,
  "temp_c":             44,
  "notes":              []
}
```

### grip() — low-level parameters
Direct access to the grip primitive — use when you need full control.
```python
result = grip(
    board,
    object_width_m  = 0.035,  # estimated object width in metres
    stall_threshold = 40,     # use STALL_THRESHOLD, STALL_THRESHOLD_FRAGILE, or STALL_THRESHOLD_MAX
    temp_limit_c    = 60,     # abort if servo exceeds this temperature
    pre_open_margin = 0.012,  # open this much wider than estimate before closing
)
# result.success       → True if contact detected
# result.final_width_m → jaw width at stop (metres)
# result.stop_reason   → "stall" | "temp_limit" | "max_pulse" | "low_voltage" | "read_error"
```

### Stall threshold tiers

| Constant | Value | Use case |
|---|---|---|
| `STALL_THRESHOLD_FRAGILE` | 30 | Fragile or compressible objects (fruit, foam, thin shells) |
| `STALL_THRESHOLD` | **40** | **Nominal — use for all standard objects** |
| `STALL_THRESHOLD_MAX` | 60 | Rigid/semi-rigid only |

### Calibration workflow

**Phase 1 — Pulse → width mapping**
```bash
python3 phase1_characterize.py  # run from grip/
```
Moves to 9 pulse positions.  At each stop, measure the jaw gap with calipers.
Saves to `phase1_results.json`.  `gripper.py` loads this at import time via
`gripper_mapping()` and uses linear interpolation instead of the theoretical model.

**Phase 2 — Stall threshold tuning**
```bash
python3 phase2_stall_threshold.py  # run from grip/
```
Closes on empty jaws (noise floor) then on objects of increasing stiffness.
Records position error at each step.  Saves to `phase2_results.json`.
Set `STALL_THRESHOLD` in `gripper.py` based on the recommended value output.

**Phase 3 — Grip security / lift test** *(in progress)*
```bash
python3 phase3_lift_test.py  # run from grip/
```
Closes slowly (step=5, half speed) at the nominal threshold, lifts the arm
via shoulder servo (servo 5 +80 pulses), monitors gripper position for slip,
then lowers and releases.  Results appended to `phase3_results.json`.

Goal: confirm threshold=40 provides enough grip force to hold each object
during arm movement, or identify the minimum threshold that does.

### Safety notes
- No current/force sensor on LX-224 servos — stall detection is position-error based
- Temperature is sampled every 5 steps as a secondary overload guard
- For force sensing upgrades: Teensy + FSR on jaw tip → USB serial to Pi
- Reference for ROS2 integration: `ArmPi_Ultra_Resources/Source Code/ROS2/src/large_models/large_models/intelligent_grasp.py`

---

## Vision Pipeline

Planning document: `selfPlanning/robot_vision_pipeline.docx`

### scripts/floor_filter.py — Stage 1: depth pre-filter

Subscribes to the Aurora 930 RGB + depth streams, fits a floor plane using
least squares (SVD), masks pixels 5–100mm above the floor, and overlays the
result on the RGB image with `cv2.imshow`.

**Run (Aurora driver must be running first):**
```bash
cd ~/ros2arm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/floor_filter.py
```

**Tunable parameters (top of file):**
| Parameter | Default | Meaning |
|---|---|---|
| `MAX_HEIGHT_M` | `0.10` | Upper height cutoff above floor (metres) |
| `SUBSAMPLE` | `8` | Use every Nth pixel for plane fitting |
| `PLANE_AVG_N` | `5` | Frames to average the floor plane over |

**What you see:**
- Green overlay = candidate pixels 5–100mm above the floor (objects)
- No green = floor or background
- On-screen text shows floor normal vector and candidate pixel count

**Notes:**
- Camera is forward-angled — floor normal is corrected to negative Y (camera frame)
- Plane is averaged over the last 5 frames for stability
- Candidate count is pixels, not objects — YOLO (Stage 3) turns pixels into bounding boxes

---

## Kinematics Notes

### TCP offset — fixed, not dynamic
`transform.py` uses a fixed `tool_link = 0.115m` for the gripper length (plus `link3 = 0.055m` for the wrist segment).
The IK target position is at the gripper tip — no manual offset needed when passing camera-derived positions.

**Open question:** the `tool_link` offset is fixed regardless of gripper aperture.
If the gripper is a parallel jaw type, this is fine (fingers move laterally, tip depth doesn't change).
If it's a linkage/scissor type, closing pulls the tip backward and the fixed offset will put the tip short of the target.
**Revisit this when integrating camera-guided grasping** — a quick empirical test (command to known position, close, check contact) will confirm whether compensation is needed.
