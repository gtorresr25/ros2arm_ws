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
| Record demonstration episodes | **Done** |
| Convert bags to HDF5 | **Done** |
| GPU training | **Next** |
| Pi inference | **Next** |

---

## Hardware

| Component | Details |
|---|---|
| Computer | Raspberry Pi 5 |
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Arm | ArmPi Ultra — 6-DOF serial bus servo arm |
| Arm connection | `/dev/ttyUSB0` at 1 Mbaud (LX-series servos) |
| Camera | Deptrum Aurora 930 (RGB only — depth disabled) |
| Camera FPS | 12 Hz |

---

## Dataset (current — as of 2026-05-30)

| Field | Value |
|---|---|
| Raw bags | `data/pick_place/episode_0`, `episode_1`, `episode_2` |
| HDF5 episodes | 48 (numbered `episode_0` … `episode_47`) |
| Image resolution | 200 × 320 RGB uint8 |
| Episode length | min 227 / mean 313 / max 407 frames |
| Episode duration | ~19 – 34 s at 12 Hz |

**Before transferring to GPU machine:** delete the leftover outlier file:
```bash
rm ~/ros2arm_ws/data/hdf5/episode_26_REMOVED.hdf5
```
That file is the original `episode_26` (502 frames, outlier) which was replaced by
the renamed `episode_48` (354 frames). The real 48 episodes are 0–47.

### Episode length note

`episode_26_REMOVED.hdf5` aside, one episode (`episode_36`, 407 frames ≈ 34 s) is
notably longer than the rest. It is within 2 σ and is included in training.

---

## Repository layout

```
ros2arm_ws/
  scripts/
    teleop_ik_v2.py          ← main teleop — runs during recording
    crosshair_overlay.py     ← ROS node: draws crosshair on RGB, republishes
    bag_to_hdf5.py           ← converts rosbags → ACT HDF5 episodes
    ros_robot_controller_sdk.py  ← vendor serial SDK (local copy)
    selfPlanning/
      ACT_handoff.md         ← this file
      armpi_constants.py     ← drop into ACT repo root
      armpi_utils.py         ← drop into ACT repo root

  data/
    pick_place/              ← raw rosbags (episode_0, episode_1, episode_2)
    hdf5/                    ← converted HDF5 episodes + norm_bounds.npz

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

## Teleop framework

`teleop_ik_v2.py` runs the arm at 12 Hz:

1. Connects to arm over serial (`Board` SDK from `ros_robot_controller_sdk.py`)
2. Moves to home1 on startup
3. **Input loop** — reads mouse/keyboard, updates desired IK state (`des_*`)
4. **12 Hz command loop**:
   - Exponentially smooths `des_*` → `smo_*` (α = 0.4)
   - Calls `kinematics.ik.solve(theta, pitch, radius, up_down, gripper_tilt=tilt, gripper=gripper_frac)`
   - Sends pulse commands to all 6 servos
   - Publishes `/joint_states`, `/teleop/ik_state`, `/teleop/recording`

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

---

## ROS 2 topics during teleop

| Topic | Type | Hz | Content |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 12 | URDF joint angles |
| `/teleop/ik_state` | `std_msgs/Float32MultiArray` | 12 | `[theta, pitch, radius, up_down, gripper_frac, tilt]` — smoothed absolute state |
| `/teleop/recording` | `std_msgs/Bool` | 12 | True while recording |
| `/aurora/rgb/crosshair` | `sensor_msgs/Image` | 12 | RGB + green crosshair |

### `/teleop/ik_state` field order
```
index 0 — theta        base yaw (rad)
index 1 — pitch        tool tilt from horizontal (rad)
index 2 — radius       reach along camera axis (m)
index 3 — up_down      vertical offset perpendicular to camera axis (m)
index 4 — gripper_frac 0.0 = fully open, 1.0 = fully closed
index 5 — tilt         wrist roll (rad)
```

---

## Observation and action space

### Observation (ACT inputs)

| Field | Source | HDF5 path | Shape | Dtype |
|---|---|---|---|---|
| RGB image | `/aurora/rgb/crosshair` | `observations/images/top` | `(T, H, W, 3)` | uint8 |
| Proprioception | `/teleop/ik_state` | `observations/qpos` | `(T, 6)` | float32 |

### Action (ACT outputs)

| Field | HDF5 path | Shape | Notes |
|---|---|---|---|
| Relative IK delta | `action` | `(T, 6)` | normalized delta |

**Actions are relative deltas, not absolute targets:**
```
action[t] = qpos[t+1] - qpos[t]   (raw, before normalization)
action[-1] = 0
```

At inference, apply each predicted (denormalized) delta to the current IK state
and pass the result to `kinematics.ik.solve()`.

### Gripper at inference

`gripper_frac` is continuous (0.0–1.0). The policy predicts a delta to apply
each tick. No stall-detection logic — just send the accumulated `gripper_frac`
directly to `solve()` as the `gripper` argument.

---

## Normalization

Fixed physical bounds — stable across datasets, no data-driven stats needed.

| Dimension | lo | hi |
|---|---|---|
| `theta` | −110° (−1.920 rad) | +110° (+1.920 rad) |
| `pitch` | −90° (−1.571 rad) | +45° (+0.785 rad) |
| `radius` | 0.00 m | 0.35 m |
| `up_down` | −0.15 m | +0.15 m |
| `gripper_frac` | 0.0 | 1.0 |
| `tilt` | −120° (−2.094 rad) | +120° (+2.094 rad) |

```python
qpos_norm   = (qpos - lo) / (hi - lo) * 2 - 1    # → [-1, 1]
action_norm = action / ((hi - lo) / 2)            # → same scale
```

Bounds are saved to `data/hdf5/norm_bounds.npz` (arrays `lo`, `hi`, `names`).

**At inference**, denormalize the policy output before applying:
```python
import numpy as np
bounds = np.load('norm_bounds.npz')
lo, hi = bounds['lo'], bounds['hi']
half_range = (hi - lo) / 2.0

# policy outputs action_norm (6,)
action_raw = action_norm * half_range          # raw delta
ik_state  += action_raw                        # apply to current state
ik_state   = np.clip(ik_state, lo, hi)        # safety clamp
```

---

## HDF5 layout

```
episode_N.hdf5
  observations/
    images/
      top/       (T, H, W, 3)  uint8   RGB with crosshair
    qpos/        (T, 6)        float32 normalized absolute state  [-1, 1]
    qpos_raw/    (T, 6)        float32 raw absolute state
  action/        (T, 6)        float32 normalized relative delta
  action_raw/    (T, 6)        float32 raw relative delta
  attrs:
    episode_len  int
    hz           12

norm_bounds.npz
  lo    (6,)  float32
  hi    (6,)  float32
  names (6,)  str
```

---

## GPU training

### 1. Clean up and transfer dataset to GPU machine

```bash
# Remove the leftover outlier file (if not already done)
rm ~/ros2arm_ws/data/hdf5/episode_26_REMOVED.hdf5

# Transfer
scp -r ~/ros2arm_ws/data/hdf5/ user@gpu-machine:~/act/data/pick_place/
scp ~/ros2arm_ws/scripts/selfPlanning/armpi_constants.py user@gpu-machine:~/act/
scp ~/ros2arm_ws/scripts/selfPlanning/armpi_utils.py     user@gpu-machine:~/act/
```

### 2. Set up ACT on GPU machine

```bash
git clone https://github.com/tonyzhaozh/act.git
cd act
pip install -r requirements.txt
```

Drop in the custom files (already transferred). In `imitate_episodes.py` and `train.py`:
```python
# from constants import ...  →  from armpi_constants import ...
# from utils import ...      →  from armpi_utils import ...
```

### 3. Set episode count and max length in armpi_constants.py

`armpi_constants.py` contains `get_task_config(task_name)` which auto-detects
`num_episodes` and `episode_len` by reading the HDF5 files on disk. However,
`armpi_utils.load_data()` currently reads from `TASK_CONFIGS` directly. Two options:

**Option A — update load_data() to use auto-detection (recommended):**

In `armpi_utils.py`, change line in `load_data()`:
```python
# Before:
cfg = TASK_CONFIGS[task_name]
# After:
from armpi_constants import get_task_config
cfg = get_task_config(task_name)
```

**Option B — hardcode the known values in armpi_constants.py:**

```python
TASK_CONFIGS = {
    'pick_place': {
        'num_episodes': 48,    # episodes 0–47
        'episode_len':  407,   # max frames (episode_36)
        ...
    }
}
```

### 4. Train — embedded model (Pi 5 deployable)

```bash
cd ~/act
python3 imitate_episodes.py \
  --task_name pick_place \
  --ckpt_dir checkpoints/pick_place_small \
  --policy_class ACT \
  --kl_weight 10 \
  --chunk_size 25 \
  --hidden_dim 256 \
  --batch_size 8 \
  --dim_feedforward 1024 \
  --enc_layers 2 \
  --dec_layers 4 \
  --nheads 4 \
  --num_epochs 2000 \
  --lr 1e-5 \
  --seed 0
```

Key parameters:
- `chunk_size 25` — ~2 s horizon at 12 Hz
- `hidden_dim 256` + `dim_feedforward 1024` — deployable on Pi 5
- `camera_names ['top']` — RGB only, no depth

### 5. Transfer checkpoint to Pi

```bash
scp -r user@gpu-machine:~/act/checkpoints/pick_place_small/ \
    ~/ros2arm_ws/checkpoints/pick_place_small/
```

---

## Pi inference

### POLICY_CONFIG (must match training flags exactly)

```python
POLICY_CONFIG = {
    'num_queries':     25,       # chunk_size
    'hidden_dim':      256,
    'dim_feedforward': 1024,
    'enc_layers':      2,
    'dec_layers':      4,
    'nheads':          4,
    'backbone':        'resnet18',
    'camera_names':    ['top'],
    'state_dim':       6,
    'action_dim':      6,
    'lr':              1e-5,
    'lr_backbone':     1e-5,
    'kl_weight':       10,
}
```

### Action execution at inference

Policy outputs `action_norm` — a normalized relative delta. Apply per tick:

```python
# Denormalize
action_raw = denormalize_action(action_norm)   # from armpi_utils

# Apply delta to current IK state
ik_state += action_raw
ik_state  = np.clip(ik_state, NORM_LO, NORM_HI)

# Unpack
theta, pitch, radius, up_down, gripper_frac, tilt = ik_state

# Send to arm
result = solve(theta, pitch, radius, up_down,
               gripper_tilt=tilt, gripper=gripper_frac)
if result.reachable:
    board.bus_servo_set_position(MOVE_DURATION, [...])
```

Apply the same exponential smoothing (α = 0.4) to `theta, pitch, radius, up_down, tilt`
before passing to `solve()` — mirrors teleop behavior and reduces jerkiness.

### Action buffering (chunk offset)

When inference finishes, skip the actions that elapsed during inference:
```python
offset = min(current_t - t_inference_start, CHUNK_SIZE - 1)
# execute chunk[offset], chunk[offset+1], ...
```

Healthy: `offset` < 50% of `chunk_size`. If consistently > 80%, the buffer risks
exhausting — use a smaller model or reduce chunk_size.

---

## Critical path

```
[Done] Record episodes
[Done] python3 scripts/bag_to_hdf5.py
[Done] Verify HDF5 quality (lengths, qpos ranges, action ranges)

[Next] rm data/hdf5/episode_26_REMOVED.hdf5
[Next] Fix load_data() in armpi_utils.py (Option A) OR hardcode counts (Option B)
[Next] Transfer dataset + custom files to GPU machine
[Next] Train with embedded config (see Step 4 above)
[Next] Transfer checkpoint to Pi
[Next] Update POLICY_CONFIG in act_policy.py
[Next] Run inference → evaluate
[Next] If poor: record more episodes, retrain
```
