#!/usr/bin/env python3
"""
ik_tcp.py — Analytical IK for the ArmPi Ultra, TCP-pitch mode.

Control parameters
------------------
theta        : base yaw (radians)                              joint1 / S6
pitch        : camera/tool pitch from horizontal (radians)     rotates AT the TCP
               positive = angled upward, 0 = horizontal, -π/2 = pointing down
r_tcp        : horizontal reach of TCP from base axis (metres) world frame
z_tcp        : height of TCP above ground (metres)             world frame
gripper_tilt : wrist roll (radians)                            wrist joint / S2
gripper      : open/close fraction  0.0 = fully open, 1.0 = fully closed

TCP-pitch geometry
------------------
r_tcp and z_tcp fix the tool tip in world space.
Changing pitch rotates the arm configuration around the TCP — the tip stays planted.

Internally converts to camera-frame (radius, up_down) via:
    z_rel   = z_tcp - d_base
    radius  =  r_tcp * cos(pitch) + z_rel * sin(pitch)
    up_down = -r_tcp * sin(pitch) + z_rel * cos(pitch)

Then runs the same geometric IK as ik.py.

Link lengths (from transform.py — definitive values)
-----------------------------------------------------
    d_base  = 0.094605 m   shoulder pivot height above ground
    L1      = 0.10048  m   upper arm      (joint2 → joint3)
    L2      = 0.100    m   forearm        (joint3 → joint4)
    L3      = 0.055    m   wrist segment  (joint4 → wrist joint)
    L_tool  = 0.115    m   gripper        (wrist joint → TCP)
    L_tcp   = 0.170    m   joint4 → TCP   (= L3 + L_tool)
"""

import math
from dataclasses import dataclass

from .transform import (
    _map,
    joint1_map, joint2_map, joint3_map, joint4_map, joint5_map,
    clamp_pulses,
)
from .ik import IKResult, _joint_angles_to_pulses, _check_limits

# ── Link lengths ──────────────────────────────────────────────────────────────
D_BASE = 0.094605
L1     = 0.10048
L2     = 0.100
L3     = 0.055
L_TOOL = 0.115
L_TCP  = L3 + L_TOOL   # 0.170 m  joint4 → TCP


# ── Public API ────────────────────────────────────────────────────────────────

def solve(
    theta:         float,
    pitch:         float,
    r_tcp:         float,
    z_tcp:         float,
    gripper_tilt:  float = 0.0,
    gripper:       float = 0.0,
    elbow_up:      bool  = False,
) -> IKResult:
    """Solve IK with TCP fixed in world space and pitch rotating around it.

    Parameters
    ----------
    theta        : Base yaw in radians.  0 = forward, positive = CCW.
    pitch        : Camera/tool pitch from horizontal in radians.
                   0 = horizontal, π/2 = pointing straight up, negative = down.
    r_tcp        : Horizontal reach of TCP from base axis (metres).
    z_tcp        : Height of TCP above ground (metres).
    gripper_tilt : Wrist roll angle in radians.  0 = neutral.
    gripper      : Gripper fraction: 0.0 = fully open, 1.0 = fully closed.
    elbow_up     : True = elbow-up solution, False = elbow-down (default).

    Returns
    -------
    IKResult with .joints (URDF angles), .pulses (servo IDs), .reachable flag.
    """
    # ── Step 1: World → camera-frame conversion ───────────────────────────────
    z_rel   = z_tcp - D_BASE
    radius  =  r_tcp * math.cos(pitch) + z_rel * math.sin(pitch)
    up_down = -r_tcp * math.sin(pitch) + z_rel * math.cos(pitch)

    # ── Step 2: Joint4 (wrist pivot) position in arm plane ───────────────────
    r_j4     = r_tcp - L_TCP * math.cos(pitch)
    z_j4_rel = z_tcp - D_BASE - L_TCP * math.sin(pitch)

    # ── Step 3: 2R planar IK for joints 2 & 3 ────────────────────────────────
    D_sq   = r_j4**2 + z_j4_rel**2
    cos_q3 = (D_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
    reachable = True

    if cos_q3 < -1.0 or cos_q3 > 1.0:
        reachable = False
        cos_q3 = max(-1.0, min(1.0, cos_q3))

    q3_std = math.acos(cos_q3)
    if not elbow_up:
        q3_std = -q3_std

    gamma  = math.atan2(z_j4_rel, r_j4)
    delta  = math.atan2(L2 * math.sin(q3_std), L1 + L2 * math.cos(q3_std))
    q2_std = gamma - delta

    # ── Step 4: Convert arm-plane angles to URDF joint angles ─────────────────
    j1 = theta
    j2 = q2_std - math.pi / 2
    j3 = -q3_std
    j4 = pitch - math.pi / 2 + j3 - j2
    j5 = gripper_tilt

    # ── Step 5: Joint limit check ─────────────────────────────────────────────
    if not _check_limits(j2, j3, j4):
        reachable = False

    # ── Step 6: Build outputs ─────────────────────────────────────────────────
    joints = {
        'joint1':        j1,
        'joint2':        j2,
        'joint3':        j3,
        'joint4':        j4,
        'wrist':         j5,
        'gripper_joint': gripper * 0.785,
    }

    pulses = _joint_angles_to_pulses(j1, j2, j3, j4, j5, gripper)

    return IKResult(joints=joints, pulses=pulses, reachable=reachable)