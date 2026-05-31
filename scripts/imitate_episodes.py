"""
imitate_episodes.py — ACT training script for ArmPi Ultra.

Patched from the original tonyzhaozh/act repo:
  - Removed sim / Aloha dependencies
  - 6-DOF ArmPi state space: [theta, pitch, radius, up_down, gripper_frac, tilt]
  - Architecture flags exposed via CLI (--enc_layers, --dec_layers, --nheads)
  - --data_dir flag to point at any local HDF5 folder
  - Early stopping via --patience
  - CUDA / MPS / CPU device auto-detection
  - Can be run from anywhere — ACT repo path is set up automatically

Usage (from ros2arm_ws or anywhere):
    python3 scripts/imitate_episodes.py \
        --task_name       pick_place \
        --ckpt_dir        checkpoints/pick_place \
        --policy_class    ACT \
        --kl_weight       10 \
        --chunk_size      50 \
        --hidden_dim      256 \
        --dim_feedforward 1024 \
        --enc_layers      4 \
        --dec_layers      5 \
        --nheads          8 \
        --batch_size      8 \
        --num_epochs      2000 \
        --lr              1e-5 \
        --seed            0 \
        --data_dir        ~/ros2arm_ws/data/hdf5
"""

import os
import sys
import pickle
import argparse
from copy import deepcopy

# ── ACT repo path setup ───────────────────────────────────────────────────────
# Allows running this script from anywhere (e.g. ros2arm_ws/scripts/).
_ACT = os.path.expanduser('~/act')
sys.path.insert(0, _ACT)
sys.path.insert(0, os.path.join(_ACT, 'detr'))

# ArmPi helpers live alongside this file or in selfPlanning/
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, 'selfPlanning'))

import matplotlib.pyplot as plt
import numpy as np
import torch
from tqdm import tqdm

from armpi_constants import DT
from armpi_utils     import load_data
from utils           import compute_dict_mean, set_seed, detach_dict

# detr/main.py calls parse_args() when build_ACT_model_and_optimizer is called.
# Swap argv to a minimal valid set around policy construction, then restore.
_saved_argv = sys.argv[:]
sys.argv    = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
               '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
from policy import ACTPolicy, CNNMLPPolicy  # noqa: E402
sys.argv = _saved_argv

# ── Device ────────────────────────────────────────────────────────────────────
device = ('cuda' if torch.cuda.is_available()
          else 'mps' if torch.backends.mps.is_available()
          else 'cpu')
print(f'Device: {device}')


# ═════════════════════════════════════════════════════════════════════════════
# Policy / optimiser helpers
# ═════════════════════════════════════════════════════════════════════════════

def make_policy(policy_class, policy_config):
    # Swap argv around policy construction — detr/main.py calls parse_args()
    # inside build_ACT_model_and_optimizer.
    sys.argv = ['act', '--ckpt_dir', '.', '--policy_class', 'ACT',
                '--task_name', 'x', '--seed', '0', '--num_epochs', '1']
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config)
    elif policy_class == 'CNNMLP':
        policy = CNNMLPPolicy(policy_config)
    else:
        raise NotImplementedError(f'Unknown policy class: {policy_class}')
    sys.argv = _saved_argv
    return policy


def make_optimizer(policy_class, policy):
    return policy.configure_optimizers()


def forward_pass(data, policy):
    image_data, qpos_data, action_data, is_pad = data
    image_data  = image_data.to(device)
    qpos_data   = qpos_data.to(device)
    action_data = action_data.to(device)
    is_pad      = is_pad.to(device)
    return policy(qpos_data, image_data, action_data, is_pad)


# ═════════════════════════════════════════════════════════════════════════════
# Training loop
# ═════════════════════════════════════════════════════════════════════════════

def train_bc(train_loader, val_loader, config):
    num_epochs   = config['num_epochs']
    ckpt_dir     = config['ckpt_dir']
    seed         = config['seed']
    policy_class = config['policy_class']
    patience     = config['patience']

    set_seed(seed)
    policy    = make_policy(policy_class, config['policy_config'])
    policy.to(device)
    optimizer = make_optimizer(policy_class, policy)

    train_history      = []
    val_history        = []
    min_val_loss       = float('inf')
    best_ckpt_info     = None
    no_improve_epochs  = 0

    for epoch in tqdm(range(num_epochs)):
        # ── Validation ────────────────────────────────────────────────────────
        policy.eval()
        with torch.inference_mode():
            epoch_dicts = [forward_pass(batch, policy) for batch in val_loader]
        epoch_summary = compute_dict_mean(epoch_dicts)
        val_history.append(epoch_summary)
        epoch_val_loss = epoch_summary['loss']

        if epoch_val_loss < min_val_loss:
            min_val_loss      = epoch_val_loss
            best_ckpt_info    = (epoch, min_val_loss, deepcopy(policy.state_dict()))
            no_improve_epochs = 0
        else:
            no_improve_epochs += 1

        print(f'\nEpoch {epoch}  val={epoch_val_loss:.5f}'
              f'  (no improvement {no_improve_epochs}/{patience})')

        # ── Training ──────────────────────────────────────────────────────────
        policy.train()
        optimizer.zero_grad()
        for batch_idx, batch in enumerate(train_loader):
            loss_dict = forward_pass(batch, policy)
            loss_dict['loss'].backward()
            optimizer.step()
            optimizer.zero_grad()
            train_history.append(detach_dict(loss_dict))

        epoch_train_loss = compute_dict_mean(
            train_history[(batch_idx + 1) * epoch:(batch_idx + 1) * (epoch + 1)])['loss']
        print(f'         train={epoch_train_loss:.5f}')

        # ── Periodic checkpoint ───────────────────────────────────────────────
        if epoch % 100 == 0:
            _save_ckpt(policy, ckpt_dir, epoch, seed)
            _plot_history(train_history, val_history, epoch, ckpt_dir, seed)

        # ── Early stopping ────────────────────────────────────────────────────
        if no_improve_epochs >= patience:
            print(f'\nEarly stopping at epoch {epoch}.')
            break

    # ── Final checkpoints ─────────────────────────────────────────────────────
    _save_ckpt(policy, ckpt_dir, epoch=-1, seed=seed, tag='last')
    best_epoch, best_loss, best_state = best_ckpt_info
    _save_ckpt_state(best_state, ckpt_dir, best_epoch, seed, tag='best')
    print(f'\nTraining done — best val {best_loss:.6f} @ epoch {best_epoch}')

    _plot_history(train_history, val_history, epoch, ckpt_dir, seed)
    return best_ckpt_info


def _save_ckpt(policy, ckpt_dir, epoch, seed, tag=None):
    name = f'policy_last.ckpt' if tag == 'last' else f'policy_epoch_{epoch}_seed_{seed}.ckpt'
    torch.save(policy.state_dict(), os.path.join(ckpt_dir, name))


def _save_ckpt_state(state_dict, ckpt_dir, epoch, seed, tag='best'):
    torch.save(state_dict, os.path.join(ckpt_dir, f'policy_{tag}.ckpt'))
    torch.save(state_dict, os.path.join(ckpt_dir, f'policy_epoch_{epoch}_seed_{seed}.ckpt'))


def _plot_history(train_history, val_history, num_epochs, ckpt_dir, seed):
    for key in train_history[0]:
        path = os.path.join(ckpt_dir, f'train_val_{key}_seed_{seed}.png')
        plt.figure()
        train_vals = [s[key].item() for s in train_history]
        val_vals   = [s[key].item() for s in val_history]
        plt.plot(np.linspace(0, num_epochs - 1, len(train_vals)), train_vals, label='train')
        plt.plot(np.linspace(0, num_epochs - 1, len(val_vals)),   val_vals,   label='val')
        plt.legend()
        plt.title(key)
        plt.tight_layout()
        plt.savefig(path)
        plt.close()


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════

def main(args):
    set_seed(1)

    task_name       = args['task_name']
    ckpt_dir        = args['ckpt_dir']
    policy_class    = args['policy_class']
    batch_size      = args['batch_size']
    num_epochs      = args['num_epochs']
    data_dir        = args.get('data_dir')
    if data_dir:
        data_dir = os.path.expanduser(data_dir)

    # ── Architecture ──────────────────────────────────────────────────────────
    state_dim = 12       # IK state (6) + joint angles (6)

    policy_config = {
        'lr':              args['lr'],
        'num_queries':     args['chunk_size'],
        'kl_weight':       args['kl_weight'],
        'hidden_dim':      args['hidden_dim'],
        'dim_feedforward': args['dim_feedforward'],
        'enc_layers':      args['enc_layers'],
        'dec_layers':      args['dec_layers'],
        'nheads':          args['nheads'],
        'lr_backbone':     1e-5,
        'backbone':        'resnet18',
        'camera_names':    ['top'],
        'state_dim':       state_dim,
    }

    # ── Data ──────────────────────────────────────────────────────────────────
    train_loader, val_loader, stats = load_data(
        task_name, batch_size, batch_size, data_dir=data_dir)

    os.makedirs(ckpt_dir, exist_ok=True)
    with open(os.path.join(ckpt_dir, 'dataset_stats.pkl'), 'wb') as f:
        pickle.dump(stats, f)

    # ── Config ────────────────────────────────────────────────────────────────
    config = {
        'num_epochs':    num_epochs,
        'ckpt_dir':      ckpt_dir,
        'state_dim':     state_dim,
        'lr':            args['lr'],
        'policy_class':  policy_class,
        'policy_config': policy_config,
        'task_name':     task_name,
        'seed':          args['seed'],
        'patience':      args['patience'],
    }

    print('\n── Policy config ──────────────────────────────')
    for k, v in policy_config.items():
        print(f'  {k:20s}: {v}')
    print('───────────────────────────────────────────────\n')

    best_ckpt_info = train_bc(train_loader, val_loader, config)
    best_epoch, min_val_loss, _ = best_ckpt_info
    print(f'Best ckpt: val loss {min_val_loss:.6f} @ epoch {best_epoch}')


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train ACT on ArmPi Ultra episodes.')

    # Required
    parser.add_argument('--task_name',       required=True,  type=str)
    parser.add_argument('--ckpt_dir',        required=True,  type=str)
    parser.add_argument('--policy_class',    required=True,  type=str,
                        help='ACT or CNNMLP')
    parser.add_argument('--batch_size',      required=True,  type=int)
    parser.add_argument('--num_epochs',      required=True,  type=int)
    parser.add_argument('--lr',              required=True,  type=float)
    parser.add_argument('--seed',            required=True,  type=int)

    # Architecture (must match act_policyv2.py POLICY_CONFIG)
    parser.add_argument('--kl_weight',       default=10,     type=int)
    parser.add_argument('--chunk_size',      default=50,     type=int)
    parser.add_argument('--hidden_dim',      default=256,    type=int)
    parser.add_argument('--dim_feedforward', default=1024,   type=int)
    parser.add_argument('--enc_layers',      default=4,      type=int)
    parser.add_argument('--dec_layers',      default=5,      type=int)
    parser.add_argument('--nheads',          default=8,      type=int)

    # Dataset / training options
    parser.add_argument('--data_dir', default=None, type=str,
                        help='Path to HDF5 episode folder (overrides armpi_constants.DATA_DIR)')
    parser.add_argument('--patience', default=50,   type=int,
                        help='Early-stopping patience in epochs')

    main(vars(parser.parse_args()))
