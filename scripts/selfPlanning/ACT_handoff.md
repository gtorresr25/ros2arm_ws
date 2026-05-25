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

All code lives in `/home/andres/ros2arm_ws/`. Key files to read:

```
scripts/
  teleop_ik_v2.py          ← MAIN TELEOP — this is what runs during recording
  selfPlanning/            ← planning docs (here)

src/
  kinematics/kinematics/
    ik.py                  ← camera-frame IK solver (read this)
    ik_tcp.py              ← TCP-pitch IK solver (legacy, not used in v2)
    transform.py           ← servo pulse ↔ angle maps + limits

  armpi_ultra_description/
    urdf/armpi_ultra.urdf  ← robot model (joint names, link geometry)
    launch/
      teleop_rviz.launch.py   ← RViz without joint_state_publisher_gui
      display.launch.py       ← original (conflicts with teleop — do not use)
    rviz/arm.rviz

  deptrum-ros-driver-aurora930/
    launch_aurora930/launch/aurora930_launch.py   ← camera ROS driver

grip/
  gripper.py               ← full smart-grip module (not used by teleop v2)
```

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
   - Publishes URDF joint angles to `/joint_states` (ROS 2 topic).
4. **Gripper** runs in a background thread (servo 1 only), stall-detected.

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

| Topic | Type | Publisher | Hz |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | teleop_ik_v2.py | 12 |
| `/aurora/rgb/image_raw` | `sensor_msgs/Image` | aurora930_node | 12 |
| `/aurora/depth/image_raw` | `sensor_msgs/Image` | aurora930_node | 12 |
| `/aurora/depth/points` | `sensor_msgs/PointCloud2` | aurora930_node | 12 |

---

## Running teleop + camera simultaneously

```bash
# Terminal 1 — camera
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py

# Terminal 2 — optional RViz (no conflict with teleop)
ros2 launch armpi_ultra_description teleop_rviz.launch.py

# Terminal 3 — teleop
cd ~/ros2arm_ws && source install/setup.bash
python3 scripts/teleop_ik_v2.py
```

> `teleop_rviz.launch.py` is a trimmed version of `display.launch.py`
> that omits `joint_state_publisher_gui` (which would conflict with the
> teleop's own `/joint_states` publisher).

---

## What Claude needs to help design

1. **Observation space** — which combination of inputs to feed ACT:
   - RGB image only?
   - RGB + depth?
   - RGB + proprioception (joint angles from `/joint_states`)?
   - Wrist-mounted camera vs. fixed camera?

2. **Action space** — what the policy should output:
   - Absolute joint angles (position control)?
   - Delta joint angles per timestep?
   - Cartesian deltas (theta, pitch, radius, up_down) in IK space?
   - Servo pulses directly?

3. **Data recorder node** — a ROS 2 node (or script) that:
   - Subscribes to `/joint_states` + `/aurora/rgb/image_raw`
   - Saves synchronized episodes to HDF5 (the ACT standard format)
   - Records on keypress start/stop during teleop

4. **ACT policy interface** — a ROS 2 node that:
   - Loads a trained checkpoint
   - Subscribes to observations
   - Publishes actions back to the arm (bypassing teleop)

5. **Practical constraints on RPi 5**:
   - ACT inference is typically done on GPU — how to handle on RPi?
   - Image resolution tradeoff (Aurora 930 supports multiple modes)
   - Whether to offload training to another machine

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
      top/          ← (T, H, W, 3) uint8
    qpos/           ← (T, num_joints) float32   joint positions
    qvel/           ← (T, num_joints) float32   joint velocities (optional)
  /action/          ← (T, num_joints) float32   joint positions commanded
```
where T = number of timesteps in the episode.

---

## Key questions to resolve first

- How many joints does the policy control? (6 arm + 1 gripper = 7 total, or
  just the 4 IK knobs + gripper?)
- Is one camera sufficient, or do we need a second wrist-mounted view?
- What is the target task exactly? (e.g., pick cube from fixed location and
  place in bowl — define the scene precisely before recording)
- Where will ACT training run? (RPi 5 cannot train — needs a separate GPU machine)
