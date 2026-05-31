"""
armpi_utils.py — Dataset utilities for ArmPi Ultra ACT training.

Drop this file into the root of the ACT repo alongside utils.py.
Use in place of utils.py when training on ArmPi Ultra episodes.

Key differences from the original utils.py:
  - No qvel (not recorded)
  - No sim flag (always real robot)
  - No depth camera — RGB only
  - 6-DOF state/action space: [theta, pitch, radius, up_down, gripper_frac, tilt]
  - Actions are relative deltas, not absolute targets
  - Normalization uses fixed physical bounds (not z-score from dataset stats)
  - Episodes have variable length — padded to max_episode_len for batching
"""

import numpy as np
import torch
import os
import h5py
from torch.utils.data import DataLoader

from armpi_constants import TASK_CONFIGS, NORM_LO, NORM_HI, STATE_DIM, ACTION_DIM, get_task_config

_NORM_RANGE      = NORM_HI - NORM_LO
_NORM_HALF_RANGE = _NORM_RANGE / 2.0


def normalize_qpos(qpos: np.ndarray) -> np.ndarray:
    """Raw absolute state → [-1, 1] using physical bounds."""
    return ((qpos - NORM_LO) / _NORM_RANGE * 2.0 - 1.0).astype(np.float32)


def denormalize_qpos(qpos_norm: np.ndarray) -> np.ndarray:
    """[-1, 1] → raw absolute state."""
    return ((qpos_norm + 1.0) / 2.0 * _NORM_RANGE + NORM_LO).astype(np.float32)


def normalize_action(action: np.ndarray) -> np.ndarray:
    """Raw relative delta → normalized scale (same units as qpos_norm)."""
    return (action / _NORM_HALF_RANGE).astype(np.float32)


def denormalize_action(action_norm: np.ndarray) -> np.ndarray:
    """Normalized delta → raw relative delta."""
    return (action_norm * _NORM_HALF_RANGE).astype(np.float32)


class ArmPiEpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, camera_names, max_episode_len):
        super().__init__()
        self.episode_ids     = episode_ids
        self.dataset_dir     = dataset_dir
        self.camera_names    = camera_names
        self.max_episode_len = max_episode_len

    def __len__(self):
        return len(self.episode_ids)

    def __getitem__(self, index):
        episode_id   = self.episode_ids[index]
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')

        with h5py.File(dataset_path, 'r') as root:
            episode_len = root['/action'].shape[0]
            start_ts    = np.random.choice(episode_len)

            # Proprioception — normalized absolute state (T, 6)
            qpos = root['/observations/qpos'][start_ts]      # (6,)

            # Images at start_ts
            image_dict = {}
            for cam_name in self.camera_names:
                image_dict[cam_name] = root[f'/observations/images/{cam_name}'][start_ts]

            # Normalized relative actions from start_ts onward
            action = root['/action'][start_ts:]              # (episode_len - start_ts, 6)

        # Pad actions to max_episode_len
        action_len               = len(action)
        padded_action            = np.zeros((self.max_episode_len, ACTION_DIM), dtype=np.float32)
        padded_action[:action_len] = action
        is_pad                   = np.zeros(self.max_episode_len, dtype=bool)
        is_pad[action_len:]      = True

        # Build image stack
        all_cam_images = []
        for cam_name in self.camera_names:
            img = image_dict[cam_name].astype(np.float32)   # uint8 (H, W, 3)
            all_cam_images.append(img)

        all_cam_images = np.stack(all_cam_images, axis=0)   # (num_cams, H, W, 3)

        # Convert to tensors
        image_data  = torch.from_numpy(all_cam_images)
        qpos_data   = torch.from_numpy(qpos).float()
        action_data = torch.from_numpy(padded_action).float()
        is_pad      = torch.from_numpy(is_pad)

        # (num_cams, H, W, C) → (num_cams, C, H, W) and normalize to [0, 1]
        image_data = torch.einsum('k h w c -> k c h w', image_data) / 255.0

        return image_data, qpos_data, action_data, is_pad


def get_norm_stats(dataset_dir, num_episodes):
    """
    Returns dummy norm stats compatible with ACT's training loop.

    Normalization is already baked into the HDF5 files (bounds-based).
    We return zero mean and unit std so the training loop's normalization
    step is a no-op.
    """
    return {
        'action_mean':  np.zeros(ACTION_DIM, dtype=np.float32),
        'action_std':   np.ones(ACTION_DIM,  dtype=np.float32),
        'qpos_mean':    np.zeros(STATE_DIM,  dtype=np.float32),
        'qpos_std':     np.ones(STATE_DIM,   dtype=np.float32),
        'example_qpos': np.zeros(STATE_DIM,  dtype=np.float32),
    }


def load_data(task_name, batch_size_train, batch_size_val):
    """Load train/val dataloaders for a given task."""
    cfg             = get_task_config(task_name)
    dataset_dir     = cfg['dataset_dir']
    num_episodes    = cfg['num_episodes']
    camera_names    = cfg['camera_names']
    max_episode_len = cfg['episode_len']

    print(f'\nTask:            {task_name}')
    print(f'Dataset dir:     {dataset_dir}')
    print(f'Episodes:        {num_episodes}')
    print(f'Max episode len: {max_episode_len}')
    print(f'Cameras:         {camera_names}\n')

    shuffled  = np.random.permutation(num_episodes)
    train_ids = shuffled[:int(0.8 * num_episodes)]
    val_ids   = shuffled[int(0.8 * num_episodes):]

    norm_stats = get_norm_stats(dataset_dir, num_episodes)

    train_dataset = ArmPiEpisodicDataset(
        train_ids, dataset_dir, camera_names, max_episode_len)
    val_dataset   = ArmPiEpisodicDataset(
        val_ids,   dataset_dir, camera_names, max_episode_len)

    train_loader = DataLoader(train_dataset, batch_size=batch_size_train,
                              shuffle=True,  pin_memory=True, num_workers=2)
    val_loader   = DataLoader(val_dataset,   batch_size=batch_size_val,
                              shuffle=True,  pin_memory=True, num_workers=2)

    return train_loader, val_loader, norm_stats
