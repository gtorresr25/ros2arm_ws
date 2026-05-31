"""
armpi_utils.py — ACT data utilities for ArmPi Ultra real-robot tasks.

Drop this file into the root of the ACT repo alongside utils.py.
Use in place of utils.py when training on ArmPi Ultra episodes.

Key differences from the original utils.py:
  - No qvel (not recorded)
  - No sim flag (always real robot)
  - Episodes have variable length — padded to max_episode_len for batching
  - Depth images are uint16 (mm), normalised to [0, 1] and expanded to 3
    channels so the same ResNet encoder handles both RGB and depth inputs
  - gripper_frac is binary (0/1) — normalisation std clamp keeps this stable
  - Uses armpi_constants.py for task config
"""

import numpy as np
import torch
import os
import h5py
from torch.utils.data import DataLoader

from armpi_constants import TASK_CONFIGS

# Depth range for tabletop pick-and-place (mm).
# Matches hardware filter: minimum_filter_depth_value=10, maximum_filter_depth_value=300.
DEPTH_MIN_MM = 10.0
DEPTH_MAX_MM = 300.0


class ArmPiEpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, camera_names, norm_stats, max_episode_len):
        super().__init__()
        self.episode_ids     = episode_ids
        self.dataset_dir     = dataset_dir
        self.camera_names    = camera_names
        self.norm_stats      = norm_stats
        self.max_episode_len = max_episode_len

    def __len__(self):
        return len(self.episode_ids)

    def __getitem__(self, index):
        episode_id   = self.episode_ids[index]
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')

        with h5py.File(dataset_path, 'r') as root:
            episode_len = root['/action'].shape[0]

            # Sample a random start timestep
            start_ts = np.random.choice(episode_len)

            # Proprioception: [theta, pitch, radius, up_down, gripper_frac]
            qpos = root['/observations/qpos'][start_ts]

            # Images at start_ts
            image_dict = {}
            for cam_name in self.camera_names:
                image_dict[cam_name] = root[f'/observations/images/{cam_name}'][start_ts]

            # Actions from start_ts onward
            action = root['/action'][start_ts:]   # (episode_len - start_ts, 5)

        # Pad actions to max_episode_len so all batches have the same shape
        action_len                      = len(action)
        padded_action                   = np.zeros((self.max_episode_len, 5), dtype=np.float32)
        padded_action[:action_len]      = action
        is_pad                          = np.zeros(self.max_episode_len, dtype=bool)
        is_pad[action_len:]             = True

        # Build image stack — normalise each modality separately
        all_cam_images = []
        for cam_name in self.camera_names:
            img = image_dict[cam_name]
            if cam_name == 'depth':
                # uint16 (H, W) → float32 (H, W, 3) normalised to [0, 1]
                img = img.astype(np.float32)
                img = np.clip(img, DEPTH_MIN_MM, DEPTH_MAX_MM)
                img = (img - DEPTH_MIN_MM) / (DEPTH_MAX_MM - DEPTH_MIN_MM)
                img = np.stack([img, img, img], axis=-1)   # (H, W, 3)
            else:
                # uint8 RGB (H, W, 3) — normalised to [0, 1] below
                img = img.astype(np.float32)
            all_cam_images.append(img)

        all_cam_images = np.stack(all_cam_images, axis=0)  # (num_cams, H, W, 3)

        # Convert to tensors
        image_data  = torch.from_numpy(all_cam_images)
        qpos_data   = torch.from_numpy(qpos).float()
        action_data = torch.from_numpy(padded_action).float()
        is_pad      = torch.from_numpy(is_pad)

        # Channel-last → channel-first: (num_cams, H, W, C) → (num_cams, C, H, W)
        image_data = torch.einsum('k h w c -> k c h w', image_data)

        # Normalise RGB to [0, 1] (depth already normalised above)
        for i, cam_name in enumerate(self.camera_names):
            if cam_name != 'depth':
                image_data[i] = image_data[i] / 255.0

        # Normalise qpos and actions with dataset statistics
        action_data = (action_data - self.norm_stats['action_mean']) / self.norm_stats['action_std']
        qpos_data   = (qpos_data   - self.norm_stats['qpos_mean'])   / self.norm_stats['qpos_std']

        return image_data, qpos_data, action_data, is_pad


def get_norm_stats(dataset_dir, num_episodes):
    """Compute mean and std for qpos and action across all episodes.

    Uses cat (not stack) to handle variable-length episodes.
    """
    all_qpos_data   = []
    all_action_data = []

    for episode_idx in range(num_episodes):
        dataset_path = os.path.join(dataset_dir, f'episode_{episode_idx}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            qpos   = root['/observations/qpos'][()]    # (T_i, 5)
            action = root['/action'][()]               # (T_i, 5)
        all_qpos_data.append(torch.from_numpy(qpos))
        all_action_data.append(torch.from_numpy(action))

    # cat along time — handles variable episode lengths
    all_qpos_data   = torch.cat(all_qpos_data,   dim=0)   # (total_frames, 5)
    all_action_data = torch.cat(all_action_data, dim=0)   # (total_frames, 5)

    action_mean = all_action_data.mean(dim=0)             # (5,)
    action_std  = all_action_data.std(dim=0)              # (5,)
    action_std  = torch.clip(action_std, 1e-2, torch.inf)

    qpos_mean   = all_qpos_data.mean(dim=0)               # (5,)
    qpos_std    = all_qpos_data.std(dim=0)                # (5,)
    qpos_std    = torch.clip(qpos_std, 1e-2, torch.inf)

    stats = {
        'action_mean':  action_mean.numpy(),
        'action_std':   action_std.numpy(),
        'qpos_mean':    qpos_mean.numpy(),
        'qpos_std':     qpos_std.numpy(),
        'example_qpos': all_qpos_data[0].numpy(),
    }
    return stats


def load_data(task_name, batch_size_train, batch_size_val):
    """Load train/val dataloaders for a given task.

    Usage:
        from armpi_utils import load_data
        train_loader, val_loader, norm_stats = load_data('pick_place', 8, 8)
    """
    cfg             = TASK_CONFIGS[task_name]
    dataset_dir     = cfg['dataset_dir']
    num_episodes    = cfg['num_episodes']
    camera_names    = cfg['camera_names']
    max_episode_len = cfg['episode_len']

    print(f'\nTask:            {task_name}')
    print(f'Dataset dir:     {dataset_dir}')
    print(f'Episodes:        {num_episodes}')
    print(f'Max episode len: {max_episode_len}')
    print(f'Cameras:         {camera_names}\n')

    # 80/20 train/val split
    shuffled  = np.random.permutation(num_episodes)
    train_ids = shuffled[:int(0.8 * num_episodes)]
    val_ids   = shuffled[int(0.8 * num_episodes):]

    norm_stats = get_norm_stats(dataset_dir, num_episodes)

    train_dataset = ArmPiEpisodicDataset(
        train_ids, dataset_dir, camera_names, norm_stats, max_episode_len)
    val_dataset   = ArmPiEpisodicDataset(
        val_ids,   dataset_dir, camera_names, norm_stats, max_episode_len)

    train_loader = DataLoader(train_dataset, batch_size=batch_size_train,
                              shuffle=True,  pin_memory=True, num_workers=2)
    val_loader   = DataLoader(val_dataset,   batch_size=batch_size_val,
                              shuffle=True,  pin_memory=True, num_workers=2)

    return train_loader, val_loader, norm_stats
