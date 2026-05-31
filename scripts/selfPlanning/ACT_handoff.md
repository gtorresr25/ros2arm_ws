# ACT Imitation Learning — Project Hand-off

## Goal

Use the mouse+keyboard teleop framework to record pick-and-place demonstrations
on the ArmPi Ultra robot arm, then train an ACT (Action Chunking with Transformers)
policy to replicate the task.

---

## Status

| Step | Status |
|---|---|
| Teleop + recording pipeline | **Done** |
| bag_to_hdf5.py converter | **Done** |
| armpi_constants.py + armpi_utils.py | **Done** |
| imitate_episodes.py (training script) | **Done** |
| act_policyv2.py (inference node) | **Done** |
| First training run (6D qpos, 96 epochs) | **Done — val 0.077** |
| First inference run | **Done — arm drifted out of range** |
| Augmented proprioception (12D qpos) | **Next** |
| Re-record demonstrations | **Next** |
| Retrain | **Next** |
| Re-run inference | **Next** |

### Why re-record?

First inference exposed a design gap: the policy's only proprioceptive input was
the *commanded* smoothed IK state (`/teleop/ik_state`), not the arm's actual
position. If the arm lags behind the command (servo load, inertia), the policy
sees the wrong state and accumulated deltas drift into unreachable configurations.

**Fix:** augment qpos with actual servo joint angles read back from the bus,
giving the policy both the commanded intent and the measured reality.

---

## Hardware

| Component | Details |
|---|---|
| Computer | Raspberry Pi 5 |
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Arm | ArmPi Ultra — 6-DOF serial bus servo arm |
| Arm connection | `/dev/ttyUSB0` at 1 Mbaud (LX-series servos) |
| Camera | Deptrum Aurora 930 (wrist-mounted, RGB only — depth disabled) |
| Camera FPS | 12 Hz |

---

## Repository layout

```
ros2arm_ws/
  scripts/
    teleop_ik_v2.py          ← main teleop — runs during recording
    crosshair_overlay.py     ← ROS node: draws crosshair on RGB, republishes
    bag_to_hdf5.py           ← converts rosbags → ACT HDF5 episodes
    imitate_episodes.py      ← training script (run from ros2arm_ws)
    act_policyv2.py          ← inference node (12 Hz, Pi deployment)
    ros_robot_controller_sdk.py  ← vendor serial SDK (local copy)
    selfPlanning/
      ACT_handoff.md         ← this file
      armpi_constants.py     ← task constants + normalization bounds
      armpi_utils.py         ← dataset loader + normalize/denormalize helpers
      readme_record.md       ← episode recording workflow

  data/
    pick_place/              ← raw rosbags
    hdf5/                    ← converted HDF5 episodes + norm_bounds.npz

  checkpoints/
    pick_place/              ← training checkpoints

  act_pi_transfer/
    patched_files/           ← drop-in replacements for ~/act repo files

  src/
    kinematics/kinematics/
      ik.py                  ← camera-frame IK solver
      transform.py           ← servo pulse ↔ angle maps + limits
    armpi_ultra_description/
      launch/teleop_rviz.launch.py
      urdf/armpi_ultra.urdf
      rviz/arm_3dviz.rviz
```

---

## Policy architecture

```
Inputs
  Image  (1, 3, 200, 320)   wrist camera RGB (uint8 → /255 → ImageNet norm)
  qpos   (12,)
    [0:6]  IK state    — theta, pitch, radius, up_down, gripper_frac, tilt
    [6:12] Joint angles — j1…j6 read from servos, converted via transform.py

      ↓                ↓
  ResNet-18        Linear embed (12 → 256)
  backbone
      ↓                ↓
      └─────── concat ─┘
                ↓
      Transformer encoder/decoder
      (CVAE: enc=4, dec=5, nheads=8, hidden=256, ff=1024)
                ↓

Output
  action chunk  (50, 6)   — 50 future IK deltas at 12 Hz (~4.2 s horizon)
  [Δtheta, Δpitch, Δradius, Δup_down, Δgripper_frac, Δtilt]
```

**Key:** only the first 6 dimensions (IK state) are used as the action target.
Joint angles [6:12] are observation-only — the policy sees the real arm position
but still outputs IK-space commands fed to `ik.solve()`.

---

## Teleop framework

`teleop_ik_v2.py` runs the arm at 12 Hz:

1. Connects to arm over serial (`Board` SDK from `ros_robot_controller_sdk.py`)
2. Moves to home1 on startup
3. **Input loop** — reads mouse/keyboard, updates desired IK state (`des_*`)
4. **12 Hz command loop**:
   - Exponentially smooths `des_*` → `smo_*` (α = 0.4)
   - Calls `kinematics.ik.solve(theta, pitch, radius, up_down, gripper_tilt=tilt, gripper=gripper_frac)`
   - Sends pulse commands to all 6 servos
   - **Reads back actual servo positions** → converts to joint angles
   - Publishes `/joint_states`, `/teleop/ik_state`, `/teleop/joint_angles`, `/teleop/recording`

### Controls

| Input | DOF | Direction |
|---|---|---|
| Mouse left/right | `theta` — base yaw | — |
| Mouse up/down | `up_down` — vertical | — |
| Scroll | `radius` — reach | up=retract, down=extend |
| Q / E | `pitch` — tool tilt | Q=up, E=down |
| A / D | `tilt` — wrist roll | A=left, D=right |
| Left click | `gripper_frac` | +0.1 per click |
| Right click | `gripper_frac` | −0.1 per click |
| I / O | recording flag | I=start, O=stop |
| H | home1 reset | — |
| 1–9 | sensitivity multiplier | — |

### Joint naming (URDF ↔ servo ID)

| URDF joint | Servo | Role |
|---|---|---|
| `joint1` | S6 | base yaw |
| `joint2` | S5 | shoulder |
| `joint3` | S4 | elbow |
| `joint4` | S3 | wrist pitch |
| `wrist`  | S2 | wrist roll |
| `gripper_joint` | S1 | gripper |

### Pulse → joint angle conversion (from transform.py + verify_fk.py)

```python
import math
from kinematics.transform import _map, joint1_map, joint2_map, joint3_map, joint4_map, joint5_map

def pulses_to_joint_angles(p: dict) -> list:
    """p = {6: pulse, 5: pulse, ..., 1: pulse}
    Returns [j1, j2, j3, j4, wrist, gripper_rad] float32."""
    j1    =  math.radians(_map(p[6], joint1_map))
    j2    = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3    =  math.radians(_map(p[4], joint3_map))
    j4    = -math.radians(_map(p[3], joint4_map)) - math.pi / 2
    wrist =  math.radians(_map(p[2], joint5_map))
    # Gripper: pulse 200 (open) → 0.0 rad, pulse 680 (closed) → 0.785 rad
    gripper = (p[1] - 200) / (680 - 200) * 0.785
    return [j1, j2, j3, j4, wrist, gripper]
```

---

## ROS 2 topics during teleop

| Topic | Type | Hz | Content |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 12 | URDF joint angles |
| `/teleop/ik_state` | `std_msgs/Float32MultiArray` | 12 | `[theta, pitch, radius, up_down, gripper_frac, tilt]` |
| `/teleop/joint_angles` | `std_msgs/Float32MultiArray` | 12 | `[j1, j2, j3, j4, wrist, gripper_rad]` — actual servo readback |
| `/teleop/recording` | `std_msgs/Bool` | 12 | True while recording |
| `/aurora/rgb/crosshair` | `sensor_msgs/Image` | 12 | RGB + green crosshair |

### `/teleop/joint_angles` field order
```
index 0 — j1          base yaw      (rad)   S6
index 1 — j2          shoulder      (rad)   S5
index 2 — j3          elbow         (rad)   S4
index 3 — j4          wrist pitch   (rad)   S3
index 4 — wrist       wrist roll    (rad)   S2
index 5 — gripper_rad gripper open  (rad)   S1   0=open, 0.785=closed
```

---

## Observation and action space

### Observation (ACT inputs)

| Field | Source | HDF5 path | Shape | Dtype |
|---|---|---|---|---|
| RGB image | `/aurora/rgb/crosshair` | `observations/images/top` | `(T, H, W, 3)` | uint8 |
| Proprioception | `/teleop/ik_state` + `/teleop/joint_angles` | `observations/qpos` | `(T, 12)` | float32 |

`qpos` layout:
```
[:, 0:6]  — normalized IK state   [theta, pitch, radius, up_down, gripper_frac, tilt]
[:, 6:12] — normalized joint angles [j1, j2, j3, j4, wrist, gripper_rad]
```

### Action (ACT outputs)

| Field | HDF5 path | Shape | Notes |
|---|---|---|---|
| Relative IK delta | `action` | `(T, 6)` | normalized delta on IK state only |

**Actions are relative deltas on the IK state, not absolute targets:**
```
action[t] = ik_state[t+1] - ik_state[t]   (raw, before normalization)
action[-1] = 0
```

At inference, apply each predicted (denormalized) delta to the current IK state
and pass the result to `kinematics.ik.solve()`.

---

## Normalization

Fixed physical bounds — stable across datasets.

### IK state (qpos[:, 0:6])

| Dimension | lo | hi |
|---|---|---|
| `theta` | −110° (−1.920 rad) | +110° (+1.920 rad) |
| `pitch` | −90° (−1.571 rad) | +45° (+0.785 rad) |
| `radius` | 0.00 m | 0.35 m |
| `up_down` | −0.15 m | +0.15 m |
| `gripper_frac` | 0.0 | 1.0 |
| `tilt` | −120° (−2.094 rad) | +120° (+2.094 rad) |

### Joint angles (qpos[:, 6:12]) — from servo pulse maps (transform.py)

| Dimension | lo | hi | Source |
|---|---|---|---|
| `j1` (base) | −120° (−2.094 rad) | +120° (+2.094 rad) | joint1_map ±120° |
| `j2` (shoulder) | −90° (−1.571 rad) | +90° (+1.571 rad) | joint2_map servo [−180°,0°] → j2 ±90° |
| `j3` (elbow) | −120° (−2.094 rad) | +120° (+2.094 rad) | joint3_map ±120° |
| `j4` (wrist pitch) | −120° (−2.094 rad) | +120° (+2.094 rad) | joint4_map servo [−200°,+20°] → j4 ≈ ±110°; 120° used as safe envelope |
| `wrist` (roll) | −120° (−2.094 rad) | +120° (+2.094 rad) | joint5_map ±120° |
| `gripper_rad` | 0.0 rad | 0.785 rad | 0 = open (pulse 200), 0.785 = closed (pulse 680) |

```python
qpos_norm   = (qpos - lo) / (hi - lo) * 2 - 1    # → [-1, 1]  applied to all 12 dims
action_norm = action / ((hi - lo) / 2)            # → same scale, applied to IK 6 dims only
```

Bounds saved to `data/hdf5/norm_bounds.npz` (`lo` (12,), `hi` (12,), `names` (12,)).
Action bounds use only `lo[:6]` / `hi[:6]`.

---

## HDF5 layout

```
episode_N.hdf5
  observations/
    images/
      top/       (T, H, W, 3)  uint8   RGB with crosshair (wrist camera)
    qpos/        (T, 12)       float32 normalized [IK_state(6) | joint_angles(6)]
    qpos_raw/    (T, 12)       float32 raw [IK_state(6) | joint_angles(6)]
  action/        (T, 6)        float32 normalized relative IK delta
  action_raw/    (T, 6)        float32 raw relative IK delta
  attrs:
    episode_len  int
    hz           12

norm_bounds.npz
  lo    (12,)  float32   first 6 = IK bounds, last 6 = joint angle bounds
  hi    (12,)  float32
  names (12,)  str
```

---

## Training

Run directly from `ros2arm_ws` (no need to `cd ~/act`):

```bash
cd ~/ros2arm_ws && python3 scripts/imitate_episodes.py \
  --task_name       pick_place \
  --ckpt_dir        checkpoints/pick_place \
  --policy_class    ACT \
  --batch_size      4 \
  --num_epochs      2000 \
  --lr              1e-5 \
  --seed            0 \
  --patience        200 \
  --data_dir        data/hdf5
```

All architecture flags default to the values in `act_policyv2.py POLICY_CONFIG`.
Override only if changing architecture (then update `act_policyv2.py` to match).

| Flag | Default | Note |
|---|---|---|
| `--chunk_size` | 50 | ~4.2 s at 12 Hz |
| `--hidden_dim` | 256 | |
| `--dim_feedforward` | 1024 | |
| `--enc_layers` | 4 | |
| `--dec_layers` | 5 | |
| `--nheads` | 8 | |
| `--patience` | 50 | Increase to 200 for longer runs |

The script auto-detects CUDA → MPS → CPU.
**~50 s/epoch on Pi 5 CPU.** Run overnight or transfer to a GPU machine.

---

## Inference

```bash
# Terminal 1
cd ~/ros2arm_ws && source install/setup.bash
ros2 launch armpi_ultra_description teleop_rviz.launch.py

# Terminal 2
cd ~/ros2arm_ws && source install/setup.bash
python3 scripts/act_policyv2.py
```

The node:
1. Loads `checkpoints/pick_place/policy_best.ckpt`
2. Connects to arm, moves to home1
3. Inference thread: runs as fast as possible, fills action buffer with full chunk
4. Control timer (12 Hz): pops one delta per tick, applies to IK state, calls `ik.solve()`
5. On IK unreachable: reverts state **and clears buffer** so inference restarts fresh

### POLICY_CONFIG (must match training)

```python
POLICY_CONFIG = {
    'lr':              1e-5,
    'num_queries':     50,
    'kl_weight':       10,
    'hidden_dim':      256,
    'dim_feedforward': 1024,
    'lr_backbone':     1e-5,
    'backbone':        'resnet18',
    'enc_layers':      4,
    'dec_layers':      5,
    'nheads':          8,
    'camera_names':    ['top'],
    'state_dim':       12,         # IK(6) + joint_angles(6)
}
```

---

## Critical path

```
[Done] Teleop + recording pipeline
[Done] bag_to_hdf5.py, armpi_constants.py, armpi_utils.py
[Done] imitate_episodes.py training script
[Done] act_policyv2.py inference node
[Done] First training run (6D, 96 epochs, val=0.077)
[Done] First inference — identified proprioception gap

[Next] Update teleop_ik_v2.py — add servo readback + /teleop/joint_angles topic
[Next] Update bag_to_hdf5.py — include joint angles, 12D qpos
[Next] Update armpi_constants.py — STATE_DIM=12, joint angle bounds
[Next] Re-record ~48 demonstration episodes
[Next] python3 scripts/bag_to_hdf5.py
[Next] python3 scripts/imitate_episodes.py --patience 200
[Next] python3 scripts/act_policyv2.py → evaluate
[Next] If poor: record more episodes, retrain
```
