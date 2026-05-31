# ACT Imitation Learning — Project Hand-off

## Goal
Use the existing mouse+keyboard teleop framework to record pick-and-place
demonstrations on the ArmPi Ultra robot arm, then train an ACT
(Action Chunking with Transformers) policy to replicate the task.

---

## Status summary

| Step | Status |
|---|---|
| Teleop + recording pipeline | **Done** |
| 15 demonstration episodes recorded | **Done** |
| bag_to_hdf5.py converter | **Done** |
| HDF5 dataset (data/hdf5/, 15 episodes) | **Done** |
| armpi_constants.py + armpi_utils.py | **Done** |
| Training environment set up (Mac M1) | **Done** — see section below |
| ACT repo cloned + patched for M1/ArmPi | **Done** — see section below |
| Training (100 epochs, early stopping) | **Done** — best val loss 0.961 @ epoch 32 |
| Checkpoint transferred to Pi | **Next** |
| Install PyTorch (CPU) on Pi | **Next** |
| act_policy.py inference node | Not started |
| Test on robot | Not started |

---

## Hardware

### Robot (Raspberry Pi)
| Component | Details |
|---|---|
| Computer | Raspberry Pi 5 |
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Languages | Python 3.12, C++ |
| Arm | ArmPi Ultra — 6-DOF serial bus servo arm |
| Arm connection | `/dev/ttyUSB0` at 1 Mbaud (LX-series servos) |
| Camera | Deptrum Aurora 930 (RGB + depth, aligned) |
| Camera FPS | 12 Hz |
| Depth filter | min 10 mm, max 300 mm (set in teleop_rviz.launch.py) |

### Training machine (Mac)
| Component | Details |
|---|---|
| Computer | MacBook Air, Apple M1 |
| RAM | 16 GB |
| OS | macOS 13 (Ventura) |
| Python env | conda `act_env`, Python 3.9 |
| PyTorch | 2.8.0, MPS backend (Metal — M1 GPU) |
| ACT repo | `/Users/gatr/Projects/armpi/act/` |
| Dataset | `/Users/gatr/Projects/armpi/act_transfer/data/hdf5/` |

---

## Repository layout

### Robot (Pi) — `/home/andres/ros2arm_ws/`
```
scripts/
  teleop_ik_v2.py          ← MAIN TELEOP — runs during recording
  crosshair_overlay.py     ← ROS node: draws crosshair on RGB, republishes
  bag_to_hdf5.py           ← converts rosbags → ACT HDF5 episodes
  selfPlanning/
    ACT_handoff.md         ← this file
    armpi_constants.py     ← ArmPi task config (also copied into ACT repo)
    armpi_utils.py         ← ArmPi dataset loader (also copied into ACT repo)

data/
  pick_place/              ← raw rosbags (episode_0 … episode_4, 3 runs each)
  hdf5/                    ← converted HDF5 episodes (episode_0 … episode_14)

src/
  kinematics/kinematics/
    ik.py                  ← camera-frame IK solver (read this)
    transform.py           ← servo pulse ↔ angle maps + limits

  armpi_ultra_description/
    urdf/armpi_ultra.urdf
    launch/
      teleop_rviz.launch.py   ← MAIN LAUNCH FILE
    rviz/
      arm_3dviz.rviz

  deptrum-ros-driver-aurora930/
    launch_aurora930/launch/aurora930_launch.py
```

### Training machine (Mac) — `/Users/gatr/Projects/armpi/`
```
act/                            ← ACT repo (cloned from tonyzhaozh/act)
  detr/                         ← vision backbone submodule (installed with pip install -e .)
  armpi_constants.py            ← copied from act_transfer/scripts/selfPlanning/
  armpi_utils.py                ← copied from act_transfer/scripts/selfPlanning/
  imitate_episodes.py           ← patched for ArmPi + MPS (see patches section)
  policy.py                     ← unchanged
  checkpoints/
    pick_place/                 ← training output goes here
      policy_best.ckpt          ← best checkpoint (by val loss)
      policy_last.ckpt          ← final epoch checkpoint
      dataset_stats.pkl         ← norm stats (needed at inference)

act_transfer/
  data/hdf5/                    ← 15 HDF5 episodes (episode_0 … episode_14)
  scripts/selfPlanning/
    ACT_handoff.md              ← this file
    armpi_constants.py          ← source of truth for constants
    armpi_utils.py              ← source of truth for dataset loader
```

---

## Training environment setup (Mac M1)

```bash
# Create env
conda create -n act_env python=3.9 -y
conda activate act_env

# PyTorch with MPS (Apple Silicon — no CUDA flags)
pip install torch torchvision

# Dependencies
pip install h5py einops pyquaternion pyyaml matplotlib opencv-python ipython packaging tqdm

# Clone ACT repo with detr submodule
cd /Users/gatr/Projects/armpi
git clone --recurse-submodules https://github.com/tonyzhaozh/act.git

# Install detr as local package (must be run from inside detr/)
cd act/detr
pip install -e .

# Copy ArmPi custom files into ACT repo
cp ../act_transfer/scripts/selfPlanning/armpi_constants.py ../act/
cp ../act_transfer/scripts/selfPlanning/armpi_utils.py     ../act/
```

**Important:** always run training commands from `/Users/gatr/Projects/armpi/act/` —
not from inside `detr/`. Python finds the `detr/` package via the working directory.

---

## Patches applied to ACT repo

The original ACT repo is written for CUDA (ALOHA robot, 14 DOF, simulation).
The following files were patched to work with M1 MPS and ArmPi's 5-DOF IK space.

### `imitate_episodes.py`
| Change | Reason |
|---|---|
| `from armpi_constants import DT` | Use ArmPi constants |
| `from armpi_utils import load_data` | Use ArmPi dataset loader |
| Guarded `from sim_env import BOX_POSE` in try/except | mujoco not installed |
| `from armpi_constants import TASK_CONFIGS` in real-robot branch | Replace aloha_scripts import |
| `state_dim = 5` | ArmPi has 5 IK dims, not 14 |
| `state_dim` added to `policy_config` | So model builder receives correct dim |
| `state_dim` added to `config` dict | Passed through to train_bc |
| `load_data(task_name, batch_size_train, batch_size_val)` | Match armpi_utils signature |
| `device = 'mps' if torch.backends.mps.is_available() else 'cpu'` | M1 GPU support |
| All `.cuda()` → `.to(device)` | CUDA not available on M1 |
| Early stopping with `patience=30` added to `train_bc` | Prevent overfitting on 15 episodes |

### `detr/main.py`
| Change | Reason |
|---|---|
| `device = 'mps' if ... else 'cpu'` added to both build functions | M1 GPU support |
| `model.cuda()` → `model.to(device)` in both build functions | CUDA not available on M1 |

### `detr/models/detr_vae.py`
| Change | Reason |
|---|---|
| `nn.Linear(14, hidden_dim)` → `nn.Linear(state_dim, hidden_dim)` (3 places) | Was hardcoded for 14-DOF ALOHA |
| `state_dim = 14` → `state_dim = getattr(args, 'state_dim', 14)` (2 places) | Read from args instead of hardcoding |

---

## How the teleop works (relevant for ACT interface design)

`teleop_ik_v2.py` does the following at runtime:

1. **Connects** to the arm over serial (`Board` SDK).
2. **Moves to home** (HOME1_PULSES — servos 2–6 only, gripper untouched).
3. **Main loop at 12 Hz**:
   - Reads mouse/keyboard input (desired Cartesian deltas).
   - Exponentially smooths toward desired (α = 0.4).
   - Calls `kinematics.ik.solve(theta, pitch, radius, up_down, gripper_frac)` →
     returns per-servo pulse values + URDF joint angles.
   - Sends pulse commands to **servos 2–6 only** (arm joints). Servo 1 (gripper)
     is NOT commanded here.
   - Publishes URDF joint angles to `/joint_states`.
   - Publishes IK state to `/teleop/ik_state`.
   - Publishes recording flag to `/teleop/recording`.
4. **Gripper** runs in a **background thread** (servo 1 only), stall-detected.
   - Left click → closes until stall (contact). Updates `gripper_frac_state[0]`
     live from actual servo readback.
   - Right click → releases. Resets `gripper_frac_state[0]` to 0.0.
   - H (home) → does NOT move the gripper.

### IK control knobs (camera-frame)

| Knob | Meaning | Input |
|---|---|---|
| `theta` | base yaw (rad) | mouse left/right |
| `pitch` | tool tilt from horizontal (rad) | Q / E keys |
| `radius` | reach along tool axis (m) | scroll |
| `up_down` | offset perpendicular to tool axis (m) | mouse up/down |
| `gripper_frac` | 0.0 = open, 1.0 = closed | left / right click |

### Joint naming (URDF ↔ servo ID)

| URDF joint | Servo ID | Role |
|---|---|---|
| `joint1` | S6 | base yaw |
| `joint2` | S5 | shoulder |
| `joint3` | S4 | elbow |
| `joint4` | S3 | wrist pitch |
| `wrist`  | S2 | wrist roll |
| `gripper_joint` | S1 | gripper |

---

## ROS 2 topics published during teleop

| Topic | Type | Hz | Notes |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 12 | URDF joint angles |
| `/teleop/ik_state` | `std_msgs/Float32MultiArray` | ~10–12 | Only when reachable |
| `/teleop/recording` | `std_msgs/Bool` | ~10–12 | i = True, o = False |
| `/aurora/rgb/crosshair` | `sensor_msgs/Image` | 12 | RGB + green crosshair |
| `/aurora/depth/image_raw` | `sensor_msgs/Image` | 12 | uint16 mm |

### `/teleop/ik_state` field order
```
[theta, pitch, radius, up_down, gripper_frac]
```
`gripper_frac` reflects the **actual servo position** readback (not commanded),
updated live by the grip thread.

---

## Crosshair overlay

`scripts/crosshair_overlay.py` draws a fixed green crosshair on every RGB frame
and republishes on `/aurora/rgb/crosshair`. **Always use this topic** — never
the raw `/aurora/rgb/image_raw` — for both training data and inference.

---

## Observation and action space

### Observation space (ACT inputs)

| Field | Source topic | HDF5 path | Shape | Dtype |
|---|---|---|---|---|
| RGB image | `/aurora/rgb/crosshair` | `observations/images/top` | `(T, 200, 320, 3)` | uint8 |
| Depth image | `/aurora/depth/image_raw` | `observations/images/depth` | `(T, 200, 320)` | uint16 |
| Proprioception | `/teleop/ik_state` | `observations/qpos` | `(T, 5)` | float32 |

### Action space (ACT outputs)

| Field | HDF5 path | Shape | Notes |
|---|---|---|---|
| IK targets | `action` | `(T, 5)` | identical to qpos |

**Actions are absolute IK knob values** `(theta, pitch, radius, up_down, gripper_frac)`.
At inference, pass directly to `kinematics.ik.solve()` for servos 2–6.

### Gripper design decision
`gripper_frac` is **binary: 0.0 = open, 1.0 = closed** in the HDF5.
The policy learns *when* to grip/release. The *how* is handled by the
stall-detection thread at inference — do not do direct position control of servo 1.

**At inference:** if `action[4] > 0.5` → trigger `grip_until_stall` thread;
else → `grip_release`. Mirror the teleop gripper logic exactly.

### IK state bounds (measured from dataset)

| Dimension | Min | Max |
|---|---|---|
| theta | -0.75 rad | 1.26 rad |
| pitch | -1.14 rad | 0.11 rad |
| radius | 0.03 m | 0.31 m |
| up_down | 0.07 m | 0.16 m |
| gripper_frac | 0.0 | 1.0 |

---

## Dataset

- **Location on Pi:** `/home/andres/ros2arm_ws/data/hdf5/`
- **Location on Mac:** `/Users/gatr/Projects/armpi/act_transfer/data/hdf5/`
- **Episodes:** 15 (`episode_0.hdf5` … `episode_14.hdf5`)
- **Source bags:** 5 bags × 3 runs each = 15 episodes
- **Episode lengths:** 380–574 frames (~32–48 s at 12 Hz)
- **Total frames:** 7250
- **Gripper:** binary (0/1) — binarized at conversion time (threshold 0.5)
- **Depth:** normalized in armpi_utils.py to [0,1] using range [10, 300] mm

To record more episodes see `scripts/selfPlanning/readme_record.md`.
To reconvert bags: `python3 scripts/bag_to_hdf5.py` (auto-increments output index).

---

## ACT reference

Original paper: "Learning Fine-Grained Bimanual Manipulation with
Low-Cost Hardware" (Zhao et al., 2023).
Reference implementation: https://github.com/tonyzhaozh/act

---

## Training command

Run from `/Users/gatr/Projects/armpi/act/` with `act_env` active:

```bash
cd /Users/gatr/Projects/armpi/act
conda activate act_env

python3 imitate_episodes.py \
  --task_name pick_place \
  --ckpt_dir checkpoints/pick_place \
  --policy_class ACT \
  --kl_weight 10 \
  --chunk_size 100 \
  --hidden_dim 512 \
  --batch_size 4 \
  --dim_feedforward 3200 \
  --num_epochs 100 \
  --lr 1e-5 \
  --seed 0
```

Key parameters:
- `chunk_size 100` — policy predicts 100 steps (~8.3 s) at once
- `kl_weight 10` — from paper, controls action diversity
- `num_epochs 100` — ceiling; early stopping (patience=30) will likely trigger earlier
- `batch_size 4` — optimal for MPS on M1
- ~25 seconds/epoch on M1 MPS → max ~42 minutes if all 100 epochs run

### Checkpoints saved
| File | Contents |
|---|---|
| `policy_best.ckpt` | Best val loss checkpoint — **use this for inference** |
| `policy_last.ckpt` | Final epoch checkpoint |
| `policy_epoch_N_seed_0.ckpt` | Snapshot every 100 epochs |
| `dataset_stats.pkl` | Normalisation stats — **required at inference** |

### Transfer checkpoint to Pi
```bash
scp -r checkpoints/pick_place/ andres@<pi-ip>:~/ros2arm_ws/checkpoints/pick_place/
```

---

## What still needs to be built

### act_policy.py — ROS 2 inference node

A ROS 2 node (`scripts/act_policy.py`) that:
1. Loads `policy_best.ckpt` and `dataset_stats.pkl`
2. Subscribes to `/aurora/rgb/crosshair`, `/aurora/depth/image_raw`, `/teleop/ik_state`
3. At 12 Hz: runs the policy → predicted action chunk `(chunk_size, 5)`
4. Executes one action per tick:
   - Pass `theta, pitch, radius, up_down` to `kinematics.ik.solve()` → send to servos 2–6
   - Gripper: `if action[4] > 0.5` → `grip_until_stall` thread; else → `grip_release`
   - **Do NOT pass `gripper_frac` to `solve()` and send servo 1 — use the stall thread instead**
5. Normalise observations using `norm_stats` from `dataset_stats.pkl`
6. Denormalise policy output before executing

---

## Training results

| Metric | Value |
|---|---|
| Best val loss | 0.961 (epoch 32) |
| Early stop at | epoch 62 (patience 30) |
| Training time | ~23 minutes on M1 MPS |
| Checkpoint | `checkpoints/pick_place/policy_best.ckpt` (320 MB) |
| Norm stats | `checkpoints/pick_place/dataset_stats.pkl` |

Training curves saved in `checkpoints/pick_place/`:
- `train_val_loss_seed_0.png` — total loss
- `train_val_l1_seed_0.png` — L1 (action regression) component
- `train_val_kl_seed_0.png` — KL divergence component

---

## Setting up on the Pi (next steps)

### 1. Transfer checkpoint files from Mac

```bash
# Run on Mac
scp checkpoints/pick_place/policy_best.ckpt \
    checkpoints/pick_place/dataset_stats.pkl \
    andres@<pi-ip>:~/ros2arm_ws/checkpoints/pick_place/
```

Or copy from the zip file — both `policy_best.ckpt` and `dataset_stats.pkl` are included.

### 2. Install PyTorch on the Pi (CPU only)

```bash
# On the Pi
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install h5py einops pyquaternion
```

The Pi has no GPU — inference runs on CPU. At 12 Hz with a 5-dim action space
and chunk_size=100, CPU is fast enough.

### 3. Clone ACT repo on the Pi and apply patches

```bash
cd ~/ros2arm_ws
git clone --recurse-submodules https://github.com/tonyzhaozh/act.git
cd act/detr && pip install -e . && cd ..
cp ../scripts/selfPlanning/armpi_constants.py .
cp ../scripts/selfPlanning/armpi_utils.py .
```

Then apply the same patches as on the Mac (see **Patches applied** section above).
The patched files are also included in the zip for reference.

### 4. Build act_policy.py

A ROS 2 node (`~/ros2arm_ws/scripts/act_policy.py`) that:
1. Loads `policy_best.ckpt` and `dataset_stats.pkl`
2. Subscribes to `/aurora/rgb/crosshair`, `/aurora/depth/image_raw`, `/teleop/ik_state`
3. At 12 Hz: runs the policy → predicted action chunk `(chunk_size, 5)`
4. Executes one action per tick:
   - Pass `theta, pitch, radius, up_down` to `kinematics.ik.solve()` → send to servos 2–6
   - Gripper: `if action[4] > 0.5` → `grip_until_stall` thread; else → `grip_release`
   - **Do NOT pass `gripper_frac` to `solve()` — use the stall thread instead**
5. Normalise observations using `norm_stats` from `dataset_stats.pkl`
6. Denormalise policy output before executing

---

## Critical path (remaining)

```
Transfer checkpoint to Pi → Install PyTorch (CPU) on Pi → Build act_policy.py → Test on robot
```
