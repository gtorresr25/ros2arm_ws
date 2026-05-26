# ACT Imitation Learning — Project Hand-off

## Goal
Use the existing mouse+keyboard teleop framework to record pick-and-place
demonstrations on the ArmPi Ultra robot arm, then train an ACT
(Action Chunking with Transformers) policy to replicate the task.

---

## Hardware

| Component | Details |
|---|---|
| Computer | Raspberry Pi 5 |
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Languages | Python 3.12, C++ |
| Arm | ArmPi Ultra — 6-DOF serial bus servo arm |
| Arm connection | `/dev/ttyUSB0` at 1 Mbaud (LX-series servos) |
| Camera | Deptrum Aurora 930 (RGB + depth, aligned) |
| Camera FPS | 12 Hz (configured in launch file) |

---

## Repository layout

All code lives in `/home/andres/ros2arm_ws/`. Key files:

```
scripts/
  teleop_ik_v2.py          ← MAIN TELEOP — runs during recording
  crosshair_overlay.py     ← ROS node: draws crosshair on RGB, republishes
  selfPlanning/            ← planning docs (here)

src/
  kinematics/kinematics/
    ik.py                  ← camera-frame IK solver (read this)
    ik_tcp.py              ← TCP-pitch IK solver (legacy, not used in v2)
    transform.py           ← servo pulse ↔ angle maps + limits

  armpi_ultra_description/
    urdf/armpi_ultra.urdf  ← robot model (joint names, link geometry)
    launch/
      teleop_rviz.launch.py   ← MAIN LAUNCH FILE (camera + crosshair + RViz)
      display.launch.py       ← original (conflicts with teleop — do not use)
    rviz/
      arm_3dviz.rviz       ← RViz config: robot model + RGB crosshair + depth
      arm.rviz             ← robot model only (no camera)

  deptrum-ros-driver-aurora930/
    launch_aurora930/launch/aurora930_launch.py   ← camera driver (launched automatically)

grip/
  gripper.py               ← full smart-grip module (not used by teleop v2)
```

---

## How to run the teleop session

```bash
# Build (only needed after code changes)
cd ~/ros2arm_ws
colcon build --packages-select kinematics armpi_ultra_description
source install/setup.bash
```

```bash
# Terminal 1 — camera driver + crosshair node + RViz
ros2 launch armpi_ultra_description teleop_rviz.launch.py
```

```bash
# Terminal 2 — teleop
cd ~/ros2arm_ws && source install/setup.bash
python3 scripts/teleop_ik_v2.py
```

> The launch file starts the Aurora 930 camera driver, the crosshair overlay
> node, and RViz all together. The teleop must stay in its own terminal because
> it uses raw terminal input (mouse + keyboard via tty.setraw).

---

## How the teleop works (relevant for ACT interface design)

`teleop_ik_v2.py` does the following at runtime:

1. **Connects** to the arm over serial (`Board` SDK).
2. **Moves to home** (HOME1_PULSES).
3. **Main loop at 12 Hz**:
   - Reads mouse/keyboard input (desired Cartesian deltas).
   - Exponentially smooths toward desired (α = 0.4).
   - Calls `kinematics.ik.solve(theta, pitch, radius, up_down)` →
     returns per-servo pulse values + URDF joint angles.
   - Sends pulse commands to servos 2–6 (arm) via serial.
   - Publishes URDF joint angles to `/joint_states`.
   - Publishes IK state to `/teleop/ik_state`.
4. **Gripper** runs in a background thread (servo 1 only), stall-detected.
   Disabled when `sens == 0` (mouse-off mode).

### IK control knobs (camera-frame)

| Knob | Meaning | Mouse axis |
|---|---|---|
| `theta` | base yaw (rad) | left/right |
| `pitch` | tool tilt from horizontal (rad) | Q / E keys |
| `radius` | reach along tool axis (m) | scroll |
| `up_down` | offset perpendicular to tool axis (m) | up/down |

### Joint naming (URDF ↔ servo ID)

| URDF joint | Servo ID | Role |
|---|---|---|
| `joint1` | S6 | base yaw |
| `joint2` | S5 | shoulder |
| `joint3` | S4 | elbow |
| `joint4` | S3 | wrist pitch |
| `wrist`  | S2 | wrist roll |
| `gripper_joint` | S1 | gripper |

Servo pulse range: 0–1000. See `transform.py` for angle ↔ pulse maps.

---

## ROS 2 topics available during teleop

| Topic | Type | Publisher | Hz | Notes |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | teleop_ik_v2.py | 12 | URDF joint angles |
| `/teleop/ik_state` | `std_msgs/Float32MultiArray` | teleop_ik_v2.py | 12 | IK knobs — see below |
| `/aurora/rgb/image_raw` | `sensor_msgs/Image` | aurora930_node | 12 | Raw RGB |
| `/aurora/rgb/crosshair` | `sensor_msgs/Image` | crosshair_overlay.py | 12 | RGB + green crosshair |
| `/aurora/depth/image_raw` | `sensor_msgs/Image` | aurora930_node | 12 | Depth (uint16, mm) |
| `/aurora/points2` | `sensor_msgs/PointCloud2` | aurora930_node | 12 | Note: NOT /aurora/depth/points |

### `/teleop/ik_state` field order
```
[theta, pitch, radius, up_down, gripper_frac]
```
- `theta`        — base yaw (rad)
- `pitch`        — tool tilt (rad)
- `radius`       — reach (m)
- `up_down`      — vertical offset (m)
- `gripper_frac` — gripper open/close fraction (0.0 = open, 1.0 = closed)

---

## Crosshair overlay

`scripts/crosshair_overlay.py` sits between the camera driver and all consumers
(RViz, data recorder, ACT policy). It draws a bright green crosshair at a fixed
position on every RGB frame and republishes on `/aurora/rgb/crosshair`.

**The crosshair must be present in both training data and inference frames.**
Always use `/aurora/rgb/crosshair` — never the raw topic — as the RGB observation.

---

## Decided: observation and action space

### Observation space (ACT inputs)

| Field | Source | HDF5 path | Shape |
|---|---|---|---|
| RGB image | `/aurora/rgb/crosshair` | `observations/images/top` | `(T, H, W, 3)` uint8 |
| Depth image | `/aurora/depth/image_raw` | `observations/images/depth` | `(T, H, W)` uint16 |
| Proprioception | `/teleop/ik_state` | `observations/qpos` | `(T, 5)` float32 |

### Action space (ACT outputs)

| Field | Source | HDF5 path | Shape |
|---|---|---|---|
| IK targets | `/teleop/ik_state` | `action` | `(T, 5)` float32 |

**Action = absolute IK knob values** `(theta, pitch, radius, up_down, gripper_frac)`.
At inference, each predicted action is passed directly to `kinematics.ik.solve()` →
servo pulses. No integration or delta accumulation.

### Why IK space, not joint space
The teleop operates in IK space — the policy learns in the same vocabulary the
human operator used. 5D instead of 7D, and maps directly to `solve()` at inference.

---

## Relevant ACT reference

Original paper: "Learning Fine-Grained Bimanual Manipulation with
Low-Cost Hardware" (Zhao et al., 2023).
Reference implementation: https://github.com/tonyzhaozh/act

The standard ACT data format uses HDF5 files with this structure:
```
episode_N.hdf5
  /observations/
    images/
      top/          ← (T, H, W, 3) uint8   RGB with crosshair
      depth/        ← (T, H, W)    uint16  depth in mm
    qpos/           ← (T, 5)       float32 [theta, pitch, radius, up_down, gripper_frac]
  /action/          ← (T, 5)       float32 [theta, pitch, radius, up_down, gripper_frac]
```

---

## What still needs to be built

### 1. Data recorder (next priority)
A script (`scripts/act_recorder.py`) that:
- Subscribes to `/aurora/rgb/crosshair` + `/aurora/depth/image_raw` + `/teleop/ik_state`
- Synchronizes all three streams (message_filters.ApproximateTimeSynchronizer)
- Records on keypress (`R` = start, `S` = stop) during a live teleop session
- Saves each episode to `data/episode_N.hdf5` in the format above
- Prints episode length and file path on save

### 2. Task definition (blocker for data collection)
The scene must be fixed before recording any episodes. Define:
- Object identity (e.g. small cube, specific color)
- Object start position (fixed marker on table)
- Goal position (e.g. fixed bowl location)
- Camera mount position (must not move between sessions)

### 3. Training machine
RPi 5 cannot train ACT — a GPU machine is required.
- Clone https://github.com/tonyzhaozh/act on the GPU machine
- Transfer HDF5 episodes via scp
- Train, then transfer checkpoint back to Pi for inference

### 4. ACT inference node
A ROS 2 node (`scripts/act_policy.py`) that:
- Loads a trained ACT checkpoint
- Subscribes to `/aurora/rgb/crosshair` + `/aurora/depth/image_raw` + `/teleop/ik_state`
- Runs the policy at 12 Hz
- Calls `kinematics.ik.solve(theta, pitch, radius, up_down, gripper_frac)` on each output
- Sends servo commands directly (bypasses teleop)

---

## Critical path

```
Task definition → Record episodes → Transfer to GPU → Train → Transfer checkpoint → Inference node → Test
```

**Task definition is the current blocker.** The scene must be fully fixed before
recording episode 1. All other items (recorder, inference node) can be built in parallel.
