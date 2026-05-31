"""
armpi_constants.py — ACT constants for ArmPi Ultra real-robot tasks.

Drop this file into the root of the ACT repo alongside constants.py.
Import from here instead of constants.py in imitate_episodes.py and train.py.

Action / state space
---------------------
5 dimensions: [theta, pitch, radius, up_down, gripper_frac]
  theta        — base yaw (rad)
  pitch        — tool tilt from horizontal (rad)
  radius       — reach along camera axis (m)
  up_down      — vertical offset perpendicular to camera axis (m)
  gripper_frac — gripper binary: 0.0 = open, 1.0 = closed

These map directly to kinematics.ik.solve() — no conversion needed at inference.
At inference, threshold gripper_frac at 0.5: >0.5 → grip_until_stall, else → release.

Camera
------
'top'   — /aurora/rgb/crosshair  (RGB with green crosshair, uint8)
'depth' — /aurora/depth/image_raw (uint16, mm)

HDF5 episode format
--------------------
episode_N.hdf5
  /observations/
    images/
      top/     (T, H, W, 3)  uint8
      depth/   (T, H, W)     uint16  mm
    qpos/      (T, 5)        float32  [theta, pitch, radius, up_down, gripper_frac]
  /action/     (T, 5)        float32  [theta, pitch, radius, up_down, gripper_frac]
"""

import os

# ── Paths ─────────────────────────────────────────────────────────────────────
# Points to act_transfer/data/ relative to this file's location (act/ dir).
DATA_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'act_transfer', 'data'))

# ── Robot constants ───────────────────────────────────────────────────────────
DT = 1 / 12                     # 12 Hz control loop

STATE_DIM  = 5                  # [theta, pitch, radius, up_down, gripper_frac]
ACTION_DIM = 5                  # same space

STATE_NAMES = ['theta', 'pitch', 'radius', 'up_down', 'gripper_frac']

# ── Task configs ──────────────────────────────────────────────────────────────
TASK_CONFIGS = {
    'pick_place': {
        'dataset_dir':    DATA_DIR + '/hdf5',
        'num_episodes':   15,
        'episode_len':    574,      # max timesteps across all recorded episodes
        'camera_names':   ['top', 'depth'],
        'state_dim':      STATE_DIM,
        'action_dim':     ACTION_DIM,
    },
}

# ── IK state bounds (measured from recorded dataset) ─────────────────────────
# Min/max of each action dimension across all 15 episodes (7250 frames total).
IK_STATE_BOUNDS = {
    'theta':        (-0.75,  1.26),   # rad
    'pitch':        (-1.14,  0.11),   # rad
    'radius':       ( 0.03,  0.31),   # m
    'up_down':      ( 0.07,  0.16),   # m
    'gripper_frac': ( 0.0,   1.0),    # binary
}
