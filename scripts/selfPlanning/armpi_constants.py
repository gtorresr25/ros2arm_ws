"""
armpi_constants.py — Task constants for ArmPi Ultra ACT training.

Drop this file into the root of the ACT repo alongside constants.py.
Import from here in imitate_episodes.py and train.py.

Observation state — 12 dimensions
-----------------------------------
  qpos[:, 0:6]  — IK state   [theta, pitch, radius, up_down, gripper_frac, tilt]
  qpos[:, 6:12] — Joint angles [j1, j2, j3, j4, wrist, gripper_rad]

Action space — 6 dimensions (IK deltas only)
---------------------------------------------
  [Δtheta, Δpitch, Δradius, Δup_down, Δgripper_frac, Δtilt]

IK dims map directly to kinematics.ik.solve() — no conversion needed at inference.
Joint angles [6:12] are observation-only (servo readback reality check).

Actions are RELATIVE deltas on the IK state:
  action[t] = ik_state[t+1] - ik_state[t]
  action[-1] = 0 (last frame)

Normalization uses fixed physical bounds (not data-driven stats):
  qpos_norm   = (qpos - lo) / (hi - lo) * 2 - 1   → [-1, 1]  (all 12 dims)
  action_norm = action / ((hi - lo) / 2)            → same scale (first 6 dims only)
Bounds are loaded from norm_bounds.npz at the dataset root.

Camera
------
  'top' — /aurora/rgb/crosshair  (RGB with green crosshair, uint8)
  No depth camera.

HDF5 episode format
--------------------
  episode_N.hdf5
    /observations/
      images/
        top/      (T, H, W, 3)  uint8   normalized RGB
      qpos/       (T, 12)       float32 normalized [IK_state(6) | joint_angles(6)]
      qpos_raw/   (T, 12)       float32 raw
    /action/      (T, 6)        float32 normalized relative IK delta
    /action_raw/  (T, 6)        float32 raw relative IK delta

  norm_bounds.npz  — lo, hi arrays (12,) for denormalization at inference
"""

import glob
import math
import os
import numpy as np

try:
    import h5py
except ImportError:
    h5py = None

# ── Paths ─────────────────────────────────────────────────────────────────────
# Set this to the folder where your HDF5 episodes are stored on the GPU machine.
DATA_DIR = os.path.expanduser('~/act/data')

# ── Robot constants ───────────────────────────────────────────────────────────
DT = 1 / 12                     # 12 Hz control loop

STATE_DIM  = 12   # IK state (6) + joint angles (6)
ACTION_DIM = 12   # padded to match state_dim — only first 6 (IK deltas) are non-zero

STATE_NAMES = [
    # IK state
    'theta', 'pitch', 'radius', 'up_down', 'gripper_frac', 'tilt',
    # Joint angles (servo readback)
    'j1', 'j2', 'j3', 'j4', 'wrist', 'gripper_rad',
]

# ── Physical normalization bounds (12D) ───────────────────────────────────────
# Columns: [lo, hi] — must match bag_to_hdf5.py NORM_BOUNDS exactly.
NORM_BOUNDS = np.array([
    # IK state (6)
    [-math.radians(110),  math.radians(110)],   # theta        (rad)
    [-math.pi / 2,        math.pi / 4      ],   # pitch        (rad)
    [ 0.00,               0.35             ],   # radius       (m)
    [-0.15,               0.15             ],   # up_down      (m)
    [ 0.0,                1.0              ],   # gripper_frac
    [-math.radians(120),  math.radians(120)],   # tilt         (rad)
    # Joint angles (6) — from servo pulse maps in transform.py
    [-math.radians(120),  math.radians(120)],   # j1  base yaw    joint1_map  ±120°
    [-math.pi / 2,        math.pi / 2      ],   # j2  shoulder    joint2_map  servo [-180,0]° → j2 ±90°
    [-math.radians(120),  math.radians(120)],   # j3  elbow       joint3_map  ±120°
    [-math.radians(120),  math.radians(120)],   # j4  wrist pitch joint4_map  servo [-200,20]° → j4 ≈ ±110° (use 120° for safety)
    [-math.radians(120),  math.radians(120)],   # wrist roll      joint5_map  ±120°
    [ 0.0,                0.785            ],   # gripper_rad     0=open, 0.785=closed
], dtype=np.float32)

NORM_LO = NORM_BOUNDS[:, 0]
NORM_HI = NORM_BOUNDS[:, 1]

# ── Task configs ──────────────────────────────────────────────────────────────
TASK_CONFIGS = {
    'pick_place': {
        'dataset_dir':    DATA_DIR + '/pick_place',
        'num_episodes':   50,           # fallback — overridden by get_task_config()
        'episode_len':    600,          # fallback — overridden by get_task_config()
        'camera_names':   ['top'],
        'state_dim':      STATE_DIM,    # 12
        'action_dim':     ACTION_DIM,   # 6
    },
}


def get_task_config(task_name: str, data_dir: str = None) -> dict:
    """Return task config with num_episodes and episode_len resolved from disk.

    Counts episode_*.hdf5 files in dataset_dir and reads the maximum
    attrs['episode_len'] across all episodes.  Falls back to the static
    values in TASK_CONFIGS if the directory is missing or h5py is not
    installed.

    data_dir: optional path override — use this when the HDF5 folder is not
              at the default ~/act/data/<task_name> location.
    """
    cfg = TASK_CONFIGS[task_name].copy()
    if data_dir is not None:
        cfg['dataset_dir'] = data_dir
    dataset_dir = cfg['dataset_dir']

    hdf5_files = sorted(glob.glob(os.path.join(dataset_dir, 'episode_*.hdf5')))
    if not hdf5_files:
        return cfg

    cfg['num_episodes'] = len(hdf5_files)

    if h5py is not None:
        max_len = 0
        for f in hdf5_files:
            with h5py.File(f, 'r') as ep:
                ep_len = int(ep.attrs.get('episode_len', 0))
                if ep_len > max_len:
                    max_len = ep_len
        if max_len > 0:
            cfg['episode_len'] = max_len

    return cfg
