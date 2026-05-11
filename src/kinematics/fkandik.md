# FK / IK Development Notes
## ArmPi Ultra — `src/kinematics` package

---

## Why this package exists

The vendor ships `inverse_kinematics.so` and `forward_kinematics.so` compiled
against **Python 3.10**.  This system runs **Python 3.12** (ROS2 Jazzy /
Ubuntu 24.04 on Raspberry Pi).  Importing them fails:

```
undefined symbol: _PyUnicode_Ready
```

This package replaces them with pure-Python equivalents using **ikpy** (a
pure-Python IK library) and the robot's URDF for geometry.

---

## Hardware facts

| Item | Value |
|---|---|
| Serial port | `/dev/ttyUSB0` |
| Baud rate | `1_000_000` |
| Servo pulse range | 0 – 1000 (500 = center) |
| Torque quirk | `bus_servo_enable_torque(id, True)` = **UNLOAD** (limp); `(id, False)` = **LOAD** (hold) |

### Servo layout

| Servo ID | Joint | Notes |
|---|---|---|
| 1 | Gripper | pulse 200 = open, ~680 = closed |
| 2 | Wrist roll | |
| 3 | Wrist pitch | |
| 4 | Elbow | |
| 5 | Shoulder | |
| 6 | Base rotation | |

### Servo ↔ IK index mapping (critical)

`transform.py` functions (`pulse2angle` / `angle2pulse`) use a 5-element
vector.  The index order is **reversed** relative to servo IDs:

| Index | Servo ID | Joint | Joint map used |
|---|---|---|---|
| 0 | 6 | Base rotation | `joint1_map` |
| 1 | 5 | Shoulder | `joint2_map` |
| 2 | 4 | Elbow | `joint3_map` |
| 3 | 3 | Wrist pitch | `joint4_map` |
| 4 | 2 | Wrist roll | `joint5_map` |

Servo 1 (gripper) is **not** part of the IK chain.

Source: `kinematics_demo.py` comment and `kinematics_control.py` servo mapping
in `ArmPi_Ultra_Resources/Source Code/ROS2/src/driver/kinematics/`.

### Joint maps (pulse ↔ angle in degrees)

```python
joint1_map = [0, 1000, 500, -120,  120,   0]   # base rotation:  pulse 500 =   0°
joint2_map = [0, 1000, 500,   30, -210, -90]   # shoulder:       pulse 500 = -90°
joint3_map = [0, 1000, 500, -120,  120,   0]   # elbow:          pulse 500 =   0°
joint4_map = [0, 1000, 500,   30, -210, -90]   # wrist pitch:    pulse 500 = -90°
joint5_map = [0, 1000, 500, -120,  120,   0]   # wrist roll:     pulse 500 =   0°
```

Format: `[pulse_min, pulse_max, pulse_center, angle_at_min, angle_at_max, angle_at_center]`

### Link lengths (metres) — from vendor `transform.py`, consistent with URDF

```python
base_link  = 0.094605   # ground to shoulder pivot
link1      = 0.10048    # shoulder to elbow
link2      = 0.100      # elbow to wrist
link3      = 0.055      # wrist segment
tool_link  = 0.115      # gripper (to jaw center)
```

---

## Package file structure

```
src/kinematics/
├── fkandik.md              ← this file
├── package.xml             ← NOT YET WRITTEN
├── setup.py                ← NOT YET WRITTEN
├── setup.cfg               ← NOT YET WRITTEN
├── resource/kinematics     ← NOT YET WRITTEN
├── urdf/
│   └── armpi_ultra.urdf    ← copied from ArmPi_Ultra_Resources (SolidWorks CAD export)
└── kinematics/
    ├── __init__.py
    ├── transform.py        ← DONE — pulse ↔ angle conversions, no ROS dependency
    ├── ik.py               ← NOT YET WRITTEN
    └── fk.py               ← NOT YET WRITTEN
```

### Dependencies

- `ikpy 3.4.2` — installed via `pip install --user ikpy --break-system-packages`
- `numpy` — already on system
- No ROS stack required at runtime

---

## URDF notes

File: `urdf/armpi_ultra.urdf` — SolidWorks CAD export, 790 lines.

**Key issue:** `joint1` (base_link → link1) is exported as `fixed`.  The
physical base rotation (servo 6) is not modeled as a revolute joint.

**ikpy chain structure** (8 links total):

| ikpy index | Name | Type |
|---|---|---|
| 0 | Base link | fixed |
| 1 | joint1 | fixed (arm base plate offset from chassis) |
| 2 | joint2 | revolute — **shoulder** |
| 3 | joint3 | revolute — **elbow** |
| 4 | joint4 | revolute — **wrist pitch** |
| 5 | joint5 | revolute — **wrist roll** |
| 6 | gripper_joint | fixed |
| 7 | r_joint | fixed (gripper finger base) |

Active links mask: `[False, False, True, True, True, True, False, False]`

**Base plate offset** (from URDF joint1 fixed origin):
`xyz = (0.066956, -8.1e-5, 0.071005)` metres from base_link.
This is the position of servo 6's rotation axis in the URDF frame.
All user-facing positions should be expressed relative to this point.

**End-effector frame issue:** ikpy's chain tip is `r_joint` (a gripper finger
joint), not the jaw center.  A fixed offset needs to be added — value not yet
determined (see calibration below).

---

## Physical measurements collected

Measured by hand — gripper tip (jaw center, gripper open) relative to servo 6
rotation axis center.  X = forward, Y = left, Z = up.

| Pose | S1 | S2 | S3 | S4 | S5 | S6 | X mm | Y mm | Z mm |
|---|---|---|---|---|---|---|---|---|---|
| home2_straight_up | 230 | 499 | 502 | 496 | 510 | 504 | 0 | 0 | 330 |
| home1 | 230 | 498 | 111 | 824 | 603 | 504 | 120 | 0 | 25 |
| reach_forward | 230 | 498 | 350 | 650 | 600 | 504 | 100 | 0 | 270 |

Saved in: `scripts/fk_measurements.json`

---

## Joint rotation axes — physically observed

Observed using `scripts/joint_jog.py` from home2 (arm pointing straight up).
Convention: X = forward, Y = left, Z = up (right-handed).

| Servo | Joint | Key (+) | Physical rotation axis | URDF axis | Match? |
|---|---|---|---|---|---|
| S6 | Base rotation | `a` | +Z (CW from above) | not modeled | — |
| S5 | Shoulder | `s` | −Y | ≈ −Y | ✓ same |
| S4 | Elbow | `d` | +Y | ≈ −Y | **opposite** |
| S3 | Wrist pitch | `f` | −Y | ≈ −Y | ✓ same |
| S2 | Wrist roll | `g` | +Z (arm up = Z) | ≈ −Z | **opposite** |
| S1 | Gripper | `h` | — (closes) | not in chain | — |

Note: S2 wrist roll axis is always along the arm's long axis.  At home2 the
arm points up (+Z), so the roll axis appears as +Z.  In general it follows
the arm direction.

---

## ikpy → servo angle conversion

Derived from physical axis observations above.

### Zero-angle offsets
At home2 (arm straight up), all ikpy angles = 0.  The corresponding servo
angles are:

| ikpy joint | Servo | Servo angle at ikpy=0 | Offset |
|---|---|---|---|
| joint2 | S5 shoulder | −90° | −90° |
| joint3 | S4 elbow | 0° | 0° |
| joint4 | S3 wrist pitch | −90° | −90° |
| joint5 | S2 wrist roll | 0° | 0° |

### Sign conventions
Shoulder and wrist pitch share the same axis direction as the URDF (−Y) so
their signs agree.  Elbow and wrist roll are **physically inverted** vs the
URDF, so their ikpy angles must be negated before pulse conversion.

```
servo_angle(S5) =  ikpy_joint2_deg  − 90     # shoulder:     same sign
servo_angle(S4) = −ikpy_joint3_deg  + 0      # elbow:        negated
servo_angle(S3) =  ikpy_joint4_deg  − 90     # wrist pitch:  same sign
servo_angle(S2) = −ikpy_joint5_deg  + 0      # wrist roll:   negated
```

Then convert each servo_angle (degrees) → pulse using `transform._map(angle, jointN_map, inverse=True)`.

### Base rotation (S6) — handled separately
```
S6_angle_deg = degrees(atan2(y_target, x_target))
S6_pulse = transform._map(S6_angle_deg, joint1_map, inverse=True)
```

---

## What is known vs unknown

### Known ✓
- Pulse ↔ servo angle conversion (`transform.py`) — taken from vendor, correct
- URDF geometry (link lengths, joint axes) — from CAD, correct
- Physical measurements for 3 poses
- At home2 (all servos ~500), arm is straight up, tip at (0, 0, 330mm)
- ikpy chain structure and active link indices
- Base plate offset in URDF frame: `(0.067, 0, 0.071)` metres
- Physical rotation axis for every joint (observed with joint_jog.py)
- Sign conventions: elbow (S4) and wrist roll (S2) are inverted vs URDF

### Still unknown ✗
- **End-effector offset** — extra length from ikpy chain tip (`r_joint`) to jaw center
  - Rough estimate: ~35mm along arm axis (from home2 measurement vs ikpy output)
  - Resolve by running FK on home2 and comparing to measured (0, 0, 330mm)

---

## SDK path

```python
SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
```

Import: `from ros_robot_controller_sdk import Board`

---

## Reference files (do not edit — vendor source only)

```
ArmPi_Ultra_Resources/Source Code/ROS2/src/driver/kinematics/kinematics/
  transform.py              ← source for our transform.py
  inverse_kinematics.so     ← Python 3.10 only, broken on this system
  forward_kinematics.so     ← Python 3.10 only, broken on this system
  kinematics_demo.py        ← shows servo↔IK index mapping in comments
  kinematics_control.py     ← shows ROS2 service approach + servo mapping

ArmPi_Ultra_Resources/Source Code/ROS2/src/simulations/armpi_ultra_description/urdf/
  armpi_ultra.urdf          ← source for our urdf/armpi_ultra.urdf
```