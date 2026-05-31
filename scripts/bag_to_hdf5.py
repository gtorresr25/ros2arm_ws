#!/usr/bin/env python3
"""
bag_to_hdf5.py

Converts ros2 bag recordings to ACT-format HDF5 files.

Usage:
    python3 scripts/bag_to_hdf5.py                 # convert all episodes
    python3 scripts/bag_to_hdf5.py --episode 1     # convert episode_1 only

Input:  data/pick_place/episode_N/
Output: data/hdf5/episode_N.hdf5
        data/hdf5/norm_bounds.npz   (fixed physical bounds, for inference)

Only frames where /teleop/recording == True are included.
RGB is converted BGR→RGB.

State / action space — 6 dimensions:
  [theta, pitch, radius, up_down, gripper_frac, tilt]

qpos   — absolute IK state at each timestep
action — relative delta: action[t] = qpos[t+1] - qpos[t], action[-1] = 0

Normalization uses fixed physical bounds (not data-driven stats):
  qpos_norm   = (qpos - lo) / (hi - lo) * 2 - 1       → [-1, 1]
  action_norm = action / ((hi - lo) / 2)               → same scale

HDF5 layout (ACT format):
  /observations/images/top    (T, H, W, 3)  uint8   RGB with crosshair
  /observations/qpos          (T, 6)         float32 normalized absolute state
  /observations/qpos_raw      (T, 6)         float32 raw absolute state
  /action                     (T, 6)         float32 normalized relative delta
  /action_raw                 (T, 6)         float32 raw relative delta
  attrs: episode_len, hz
"""

import os
import math
import bisect
import argparse

import numpy as np
import h5py
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
BAGS_DIR  = os.path.join(_HERE, '..', 'data', 'pick_place')
HDF5_DIR  = os.path.join(_HERE, '..', 'data', 'hdf5')

# ── Topics ────────────────────────────────────────────────────────────────────
TOPIC_RGB       = '/aurora/rgb/crosshair'
TOPIC_IK        = '/teleop/ik_state'
TOPIC_RECORDING = '/teleop/recording'

ALL_TOPICS = {TOPIC_RGB, TOPIC_IK, TOPIC_RECORDING}

# ── Physical bounds for each IK dimension ─────────────────────────────────────
# Used for fixed-range normalization — stable across datasets.
# Columns: [lo, hi] for [theta, pitch, radius, up_down, gripper_frac, tilt]
NORM_BOUNDS = np.array([
    [-math.radians(110),  math.radians(110)],   # theta        (rad)
    [-math.pi / 2,        math.pi / 4      ],   # pitch        (rad)
    [ 0.00,               0.35             ],   # radius       (m)
    [-0.15,               0.15             ],   # up_down      (m)
    [ 0.0,                1.0              ],   # gripper_frac (fraction)
    [-math.radians(120),  math.radians(120)],   # tilt         (rad)
], dtype=np.float32)

_LO        = NORM_BOUNDS[:, 0]
_HI        = NORM_BOUNDS[:, 1]
_RANGE     = _HI - _LO           # full range per dimension
_HALF_RANGE = _RANGE / 2.0       # used to scale action deltas


# ── Normalization ─────────────────────────────────────────────────────────────

def normalize_qpos(qpos: np.ndarray) -> np.ndarray:
    """Map absolute state from physical range to [-1, 1]."""
    return ((qpos - _LO) / _RANGE * 2.0 - 1.0).astype(np.float32)


def normalize_action(action: np.ndarray) -> np.ndarray:
    """Scale relative deltas by half-range so units match normalized qpos."""
    return (action / _HALF_RANGE).astype(np.float32)


# ── Action computation ────────────────────────────────────────────────────────

def qpos_to_action(qpos_arr: np.ndarray) -> np.ndarray:
    """Relative delta: action[t] = qpos[t+1] - qpos[t], action[-1] = 0."""
    action = np.zeros_like(qpos_arr)
    action[:-1] = qpos_arr[1:] - qpos_arr[:-1]
    return action


# ── Bag reading ───────────────────────────────────────────────────────────────

def read_bag(bag_dir: str) -> dict:
    """Read all relevant messages from a bag directory."""
    storage_options   = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {
        meta.name: get_message(meta.type)
        for meta in reader.get_all_topics_and_types()
    }

    messages = {t: [] for t in ALL_TOPICS}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in ALL_TOPICS:
            msg = deserialize_message(data, type_map[topic])
            messages[topic].append((timestamp, msg))

    return messages


# ── Recording window ──────────────────────────────────────────────────────────

def recording_intervals(recording_msgs: list) -> list:
    """Return list of (start_ns, end_ns) where /teleop/recording was True."""
    intervals = []
    start = None
    for ts, msg in recording_msgs:
        if msg.data and start is None:
            start = ts
        elif not msg.data and start is not None:
            intervals.append((start, ts))
            start = None
    if start is not None:
        intervals.append((start, recording_msgs[-1][0]))
    return intervals


# ── Timestamp sync ────────────────────────────────────────────────────────────

def nearest_msg(msgs: list, ts: int):
    """Return the message whose timestamp is closest to ts."""
    if not msgs:
        return None
    timestamps = [m[0] for m in msgs]
    idx = bisect.bisect_left(timestamps, ts)
    if idx == 0:
        return msgs[0][1]
    if idx >= len(msgs):
        return msgs[-1][1]
    before, after = msgs[idx - 1], msgs[idx]
    return (before if abs(before[0] - ts) <= abs(after[0] - ts) else after)[1]


# ── Image decoding ────────────────────────────────────────────────────────────

def decode_rgb(msg) -> np.ndarray:
    """BGR8 image message → (H, W, 3) uint8 RGB."""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return arr[:, :, ::-1].copy()


# ── HDF5 writer ───────────────────────────────────────────────────────────────

def write_hdf5(rgbs, qpos_raw, out_path: str) -> None:
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    action_raw  = qpos_to_action(qpos_raw)
    qpos_norm   = normalize_qpos(qpos_raw)
    action_norm = normalize_action(action_raw)
    with h5py.File(out_path, 'w') as f:
        obs = f.create_group('observations')
        img = obs.create_group('images')
        img.create_dataset('top',       data=np.stack(rgbs), dtype=np.uint8,   compression='lzf')
        obs.create_dataset('qpos',      data=qpos_norm,      dtype=np.float32)
        obs.create_dataset('qpos_raw',  data=qpos_raw,       dtype=np.float32)
        f.create_dataset('action',      data=action_norm,    dtype=np.float32)
        f.create_dataset('action_raw',  data=action_raw,     dtype=np.float32)
        f.attrs['episode_len'] = len(rgbs)
        f.attrs['hz']          = 12


# ── Bag conversion ────────────────────────────────────────────────────────────

def convert_bag(bag_dir: str, hdf5_dir: str, out_idx: int) -> int:
    """Convert one bag, writing one HDF5 per recording segment."""
    print(f"\n  Reading {os.path.basename(bag_dir)} …")
    messages = read_bag(bag_dir)

    if not messages[TOPIC_RECORDING]:
        print("  [SKIP] No /teleop/recording messages.")
        return out_idx

    intervals = recording_intervals(messages[TOPIC_RECORDING])
    if not intervals:
        print("  [SKIP] recording flag never set to True.")
        return out_idx

    print(f"  Segments: {len(intervals)}")

    for seg_i, (seg_start, seg_end) in enumerate(intervals):
        rgbs, qposs = [], []

        for ts, rgb_msg in messages[TOPIC_RGB]:
            if not (seg_start <= ts <= seg_end):
                continue
            ik_msg = nearest_msg(messages[TOPIC_IK], ts)
            if ik_msg is None:
                continue
            rgbs.append(decode_rgb(rgb_msg))
            qposs.append(np.array(ik_msg.data, dtype=np.float32))

        T = len(rgbs)
        if T == 0:
            print(f"    Segment {seg_i}: no frames — skipping.")
            continue

        qpos_raw = np.stack(qposs)
        out_path = os.path.join(hdf5_dir, f'episode_{out_idx}.hdf5')
        write_hdf5(rgbs, qpos_raw, out_path)
        print(f"    Segment {seg_i}: {T} frames (~{T/12:.1f}s) → episode_{out_idx}.hdf5")
        out_idx += 1

    return out_idx


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Convert ros2 bags to ACT HDF5.')
    parser.add_argument('--episode', type=int, default=None,
                        help='Convert a single episode by number (default: all)')
    args = parser.parse_args()

    bags_dir = os.path.realpath(BAGS_DIR)
    hdf5_dir = os.path.realpath(HDF5_DIR)

    if args.episode is not None:
        episode_dirs = [f'episode_{args.episode}']
    else:
        episode_dirs = sorted(
            d for d in os.listdir(bags_dir)
            if d.startswith('episode_') and os.path.isdir(os.path.join(bags_dir, d))
        )

    print(f"Bags found: {len(episode_dirs)}")
    print(f"\nNorm bounds:")
    names = ['theta', 'pitch', 'radius', 'up_down', 'gripper_frac', 'tilt']
    for i, name in enumerate(names):
        print(f"  {name:14s}  [{_LO[i]:+.4f}, {_HI[i]:+.4f}]")

    out_idx = 0
    for ep_dir in episode_dirs:
        bag_path   = os.path.join(bags_dir, ep_dir)
        mcap_files = [f for f in os.listdir(bag_path) if f.endswith('.mcap')]
        if not mcap_files:
            print(f"\n  [SKIP] {ep_dir} — no .mcap file.")
            continue
        out_idx = convert_bag(bag_path, hdf5_dir, out_idx)

    # Save bounds alongside HDF5 files for use at inference
    os.makedirs(hdf5_dir, exist_ok=True)
    bounds_path = os.path.join(hdf5_dir, 'norm_bounds.npz')
    np.savez(bounds_path, lo=_LO, hi=_HI, names=names)
    print(f"\nDone. {out_idx} episode(s) written to {hdf5_dir}/")
    print(f"Norm bounds saved to {bounds_path}")


if __name__ == '__main__':
    main()
