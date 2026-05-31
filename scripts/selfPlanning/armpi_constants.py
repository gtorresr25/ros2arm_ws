"""
armpi_constants.py — Task constants for ArmPi Ultra ACT training.

Drop this file into the root of the ACT repo alongside constants.py.
Import from here in imitate_episodes.py and train.py.

Action / state space — 6 dimensions
-------------------------------------
  [theta, pitch, radius, up_down, gripper_frac, tilt]

  theta        — base yaw (rad)
  pitch        — tool tilt from horizontal (rad)
  radius       — reach along camera axis (m)
  up_down      — vertical offset perpendicular to camera axis (m)
  gripper_frac — gripper fraction: 0.0 = fully open, 1.0 = fully closed
  tilt         — wrist roll (rad)

These map directly to kinematics.ik.solve() — no conversion needed at inference.

Actions are RELATIVE deltas (not absolute positions):
  action[t] = qpos[t+1] - qpos[t]
  action[-1] = 0 (last frame)

Normalization uses fixed physical bounds (not data-driven stats):
  qpos_norm   = (qpos - lo) / (hi - lo) * 2 - 1   → [-1, 1]
  action_norm = action / ((hi - lo) / 2)            → same scale
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
      qpos/       (T, 6)        float32 normalized absolute state
      qpos_raw/   (T, 6)        float32 raw absolute state
    /action/      (T, 6)        float32 normalized relative delta
    /action_raw/  (T, 6)        float32 raw relative delta

  norm_bounds.npz  — lo, hi arrays (6,) for denormalization at inference
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

STATE_DIM  = 6
ACTION_DIM = 6

STATE_NAMES = ['theta', 'pitch', 'radius', 'up_down', 'gripper_frac', 'tilt']

# ── Physical normalization bounds ─────────────────────────────────────────────
# Columns: [lo, hi] — must match bag_to_hdf5.py NORM_BOUNDS exactly.
NORM_BOUNDS = np.array([
    [-math.radians(110),  math.radians(110)],   # theta        (rad)
    [-math.pi / 2,        math.pi / 4      ],   # pitch        (rad)
    [ 0.00,               0.35             ],   # radius       (m)
    [-0.15,               0.15             ],   # up_down      (m)
    [ 0.0,                1.0              ],   # gripper_frac
    [-math.radians(120),  math.radians(120)],   # tilt         (rad)
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
        'state_dim':      STATE_DIM,
        'action_dim':     ACTION_DIM,
    },
}


def get_task_config(task_name: str) -> dict:
    """Return task config with num_episodes and episode_len resolved from disk.

    Counts episode_*.hdf5 files in dataset_dir and reads the maximum
    attrs['episode_len'] across all episodes.  Falls back to the static
    values in TASK_CONFIGS if the directory is missing or h5py is not
    installed.

    Usage in imitate_episodes.py / train.py:
        from armpi_constants import get_task_config
        task_config = get_task_config(args.task_name)
    """
    cfg = TASK_CONFIGS[task_name].copy()
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
