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
| ID | Joint | URDF joint | transform.py map |
|---|---|---|---|
| 1 | Gripper | `gripper_joint` | linear (200–680 → 0–0.785 rad) |
| 2 | Wrist roll | `wrist` | `joint5_map` |
| 3 | Wrist pitch | `joint4` | `joint4_map` |
| 4 | Elbow | `joint3` | `joint3_map` |
| 5 | Shoulder | `joint2` | `joint2_map` |
| 6 | Base rotation | `joint1` | `joint1_map` |

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

## Mouse + Keyboard Teleop v2 (with RViz + Camera)

**Status: complete and tested on hardware.**

`teleop_ik_v2.py` — mouse-driven teleop with stall-detected gripper, RViz visualization,
and live RGB + depth camera feed. Requires a one-time build if packages have changed.

### Build
```bash
cd ~/ros2arm_ws
colcon build --packages-select kinematics armpi_ultra_description
source install/setup.bash
```

### Run
```bash
# Terminal 1 — camera driver + RViz (robot model, RGB feed, depth feed)
ros2 launch armpi_ultra_description teleop_rviz.launch.py

# Terminal 2 — teleop
cd ~/ros2arm_ws && source install/setup.bash
python3 scripts/teleop_ik_v2.py
```

### Controls
| Input | Action |
|---|---|
| Mouse left / right | Base rotation (`theta`) |
| Mouse up / down | Vertical position (`up_down`) |
| Scroll up / down | Reach in / out (`radius`) |
| `Q` / `E` | Pitch tilt up / down |
| Left click | Grip (stall-detected) |
| Right click | Release gripper |
| `H` | Return to home1 |
| `0` | Mouse off (disables all mouse input + gripper) |
| `1`–`9` | Sensitivity multiplier |
| `P` | Print current state |
| `Esc` / `Ctrl-C` | Quit |

---

## Keyboard Teleop

**Status: complete and tested on hardware.**

Two teleop scripts with identical controls, differing only in how pitch behaves.
Both start at home1 and revert any move that the IK deems unreachable.

### Files
| File | Mode | Pitch behaviour |
|---|---|---|
| `scripts/teleop_ik.py` | Camera-frame | Pitch moves the TCP along the camera axis |
| `scripts/teleop_ik_tcp.py` | TCP-pitch | Pitch rotates the tool at the TCP — tip stays fixed |

### Controls — `teleop_ik.py` (camera-frame)
| Key | Action |
|---|---|
| `W` / `S` | Up / down (`up_down`) |
| `E` / `Q` | Forward / backward (`radius`) |
| `R` / `F` | Pitch tilt up / down |
| `A` / `D` | Base rotate left / right |
| `H` | Return to home1 |
| `1`–`9` | Step size multiplier |
| `P` | Print current state |
| `Esc` / `Ctrl-C` | Quit |

### Controls — `teleop_ik_tcp.py` (TCP-pitch)
| Key | Action |
|---|---|
| `W` / `S` | Up / down (`z_tcp`) |
| `E` / `Q` | Forward / backward (`r_tcp`) |
| `R` / `F` | Pitch tilt up / down (rotates at TCP) |
| `A` / `D` | Base rotate left / right |
| `H` | Return to home1 |
| `1`–`9` | Step size multiplier |
| `P` | Print current state |
| `Esc` / `Ctrl-C` | Quit |

### Run
```bash
cd ~/ros2arm_ws
colcon build --packages-select kinematics
source install/setup.bash

python3 scripts/teleop_ik.py        # camera-frame mode
python3 scripts/teleop_ik_tcp.py    # TCP-pitch mode
```

---

## IK Visualiser

Standalone 2D stick-figure debugger — no ROS, no robot connection.
Sliders update the arm live.  Status box shows TCP position and joint angles;
turns red when out of reach.

### Files
| File | Mode | Sliders |
|---|---|---|
| `scripts/ik_viz.py` | Camera-frame | `pitch`, `radius`, `up_down` |
| `scripts/ik_viz_tcp.py` | TCP-pitch | `pitch`, `r_tcp`, `z_tcp` |

### Run
```bash
python3 scripts/ik_viz.py        # camera-frame
python3 scripts/ik_viz_tcp.py    # TCP-pitch
```

Both open at the home1 position.

---

## Kinematics Package

**Status: complete and built.**

Pure-Python analytical IK/FK for the ArmPi Ultra.
Replaces the vendor's Python-3.10-only `.so` files.

### Why we need this
The vendor's `inverse_kinematics.so` and `forward_kinematics.so` were compiled
against Python 3.10.  This system runs Python 3.12 (ROS2 Jazzy / Ubuntu 24.04).
Importing them fails with `undefined symbol: _PyUnicode_Ready`.

### Package location
```
ros2arm_ws/src/kinematics/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/kinematics
└── kinematics/
    ├── __init__.py
    ├── transform.py   ← servo ↔ angle conversions (extracted from vendor)
    ├── ik.py          ← camera-frame IK: solve(theta, pitch, radius, up_down)
    └── ik_tcp.py      ← TCP-pitch IK:    solve(theta, pitch, r_tcp, z_tcp)
```

### Build
```bash
cd ~/ros2arm_ws
colcon build --packages-select kinematics
source install/setup.bash
```

### IK approach — analytical geometry, no external library
Both solvers use the same 3-step geometric IK:
1. **Base yaw** — `theta` sets joint1 directly.
2. **2R planar IK** — solves joints 2 & 3 in the arm's vertical plane.
3. **Wrist pitch** — joint4 is derived to match the desired tool pitch.

Elbow-down solution is used (verified against all four home positions with `ik_viz.py`).

### Control modes

**Camera-frame (`ik.py`)**
```python
from kinematics.ik import solve
result = solve(theta, pitch, radius, up_down)
# radius  = reach along camera optical axis from shoulder pivot
# up_down = offset perpendicular to camera axis (positive = up)
```

**TCP-pitch (`ik_tcp.py`)**
```python
from kinematics.ik_tcp import solve
result = solve(theta, pitch, r_tcp, z_tcp)
# r_tcp = horizontal reach of TCP from base axis (world frame)
# z_tcp = height of TCP above ground (world frame)
# pitch rotates the arm AT the TCP — tip stays fixed in space
```

### IKResult fields
```python
result.joints    # {'joint1': rad, 'joint2': rad, ..., 'wrist': rad, 'gripper_joint': rad}
result.pulses    # {6: int, 5: int, 4: int, 3: int, 2: int, 1: int}  keyed by servo ID
result.reachable # False if target is outside workspace or joint limits
```

### Servo ↔ IK index mapping
| Servo ID | Joint | Role |
|---|---|---|
| S6 | `joint1` | Base rotation |
| S5 | `joint2` | Shoulder |
| S4 | `joint3` | Elbow |
| S3 | `joint4` | Wrist pitch |
| S2 | `wrist` | Wrist roll |
| S1 | `gripper_joint` | Gripper (not part of IK chain) |

### Link lengths (from transform.py — definitive)
| Symbol | Value | Segment |
|---|---|---|
| `d_base` | 0.094605 m | Ground → shoulder pivot |
| `L1` | 0.10048 m | Upper arm (joint2 → joint3) |
| `L2` | 0.100 m | Forearm (joint3 → joint4) |
| `L3` | 0.055 m | Wrist segment (joint4 → wrist joint) |
| `L_tool` | 0.115 m | Gripper (wrist joint → TCP) |
| `L_tcp` | 0.170 m | joint4 → TCP (= L3 + L_tool) |

---

## ArmPi Ultra Description Package (URDF Simulation)

**Status: URDF complete and FK-verified — definitive model for this project.**

Custom ROS2 description package with a hand-built URDF using primitive shapes
(no vendor STL meshes).  Launches a full RViz2 simulation with interactive
joint sliders.  This is the canonical robot model going forward for FK/IK work.

**FK verification complete: RViz model mirrors the physical arm via `verify_fk.py`.**

### Package location
```
ros2arm_ws/src/armpi_ultra_description/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/armpi_ultra_description
├── armpi_ultra_description/__init__.py
├── urdf/
│   └── armpi_ultra.urdf          ← definitive URDF
├── launch/
│   └── display.launch.py         ← launches RSP + joint_state_publisher_gui + RViz2
└── rviz/
    └── arm.rviz                   ← RViz2 config (Grid, RobotModel, TF)
```

### Launch
```bash
cd ~/ros2arm_ws
colcon build --packages-select armpi_ultra_description
source install/setup.bash
ros2 launch armpi_ultra_description display.launch.py
```

### FK verification (live robot → RViz)
```bash
# Terminal 1
ros2 launch armpi_ultra_description display.launch.py
# Terminal 2 — close the slider GUI window first to avoid conflict
source ~/ros2arm_ws/install/setup.bash
python3 scripts/verify_fk.py
```
`verify_fk.py` reads the physical arm via the SDK, converts servo pulses to joint
angles, and publishes to `/joint_states`.  The RViz model mirrors the real arm live.
Close the `joint_state_publisher_gui` window to hand control to the script.

### URDF joint chain

| Joint name | Type | Axis | Limits | Servo | Sign convention |
|---|---|---|---|---|---|
| `joint1` | revolute | Z | ±120° | S6 — base rotation | direct |
| `joint2` | revolute | −Y | ±90° | S5 — shoulder | flipped, −90° offset |
| `joint3` | revolute | +Y | ±90° | S4 — elbow | direct |
| `joint4` | revolute | −Y | ±120° | S3 — wrist pitch | flipped, −90° offset |
| `wrist` | revolute | +Z | ±90° | S2 — wrist roll | direct |
| `fixed_finger_joint` | fixed | — | — | — | — |
| `gripper_joint` | revolute | +X | 0°–45° | S1 — gripper | inverted (open=max) |

### Link geometry

| Link | Shape | Dimensions (x × y × z mm) | Color |
|---|---|---|---|
| `base_link` | cylinder | r=60, h=95 | dark grey |
| `link1` | — | zero-length connector | — |
| `link2` | box | 30 × 55 × 100 | blue-grey |
| `link3` | box | 22 × 40 × 95 | light grey |
| `link4` | box | 35 × 40 × 50 | blue-grey |
| `link5` | box | 15 × 50 × 5 (palm plate) | dark grey |
| `fixed_finger` | box | 7.5 × 10 × 90 | yellow |
| `gripper_finger` | box | 7.5 × 10 × 90 | orange |

### Gripper convention
- `gripper_joint = 0.0 rad` → fully open
- `gripper_joint = 0.785 rad (45°)` → fully closed
- Positive rotation around +X closes the finger toward the fixed finger.

### Design notes
- Vendor URDF uses broken `package://44 001/meshes/...` URIs — avoided entirely.
- `link1` is a zero-length link connecting `joint1` (Z rotation) and `joint2` (Y rotation) at the same position.
- Wrist roll (servo 2) is not yet modeled — deferred to a future iteration.
- Adams apple (servo housing on link5) is deferred cosmetic detail.

---

## Kinematics Notes

### TCP offset — fixed, not dynamic
`transform.py` uses a fixed `L_tool = 0.115m` for the gripper length (plus `L3 = 0.055m` for the wrist segment).
The IK target is always the gripper tip — no manual offset needed.

**Open question:** the `L_tool` offset is fixed regardless of gripper aperture.
If the gripper is a parallel jaw type, this is fine (fingers move laterally, tip depth doesn't change).
If it's a linkage/scissor type, closing pulls the tip backward and the fixed offset will put the tip short of the target.
Revisit when integrating camera-guided grasping.

### Angle sign conventions (pulse → URDF)
```
j1 =  radians(_map(p6, joint1_map))
j2 = -radians(_map(p5, joint2_map)) - π/2
j3 =  radians(_map(p4, joint3_map))
j4 = -radians(_map(p3, joint4_map)) - π/2
j5 =  radians(_map(p2, joint5_map))
```
j2 and j4 are flipped and offset by −90°.  At servo center (pulse 500) both read 0° → URDF angle = −π/2.

### Elbow solution
Elbow-down (`elbow_up=False`) is the correct solution for this arm.
Verified visually with `ik_viz.py` against all four home positions.
