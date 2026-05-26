#!/usr/bin/env python3
"""
ik.py — Analytical inverse kinematics for the ArmPi Ultra.

Control parameters
------------------
theta        : base yaw (radians)                              joint1 / S6
pitch        : camera/tool pitch from horizontal (radians)     cumulative j2+j3+j4
               positive = angled upward, 0 = horizontal, -π/2 = pointing down
radius       : reach along camera optical axis from shoulder pivot (metres)
up_down      : offset perpendicular to camera axis, positive = upward (metres)
gripper_tilt : wrist roll (radians)                            wrist joint / S2
gripper      : open/close fraction  0.0 = fully open, 1.0 = fully closed

Camera-frame geometry
---------------------
Origin      : shoulder pivot — the point where joint1 and joint2 share the same
              position at height d_base above the ground.
Forward     : along the camera/tool pointing direction at angle `pitch` from horizontal.
Up          : perpendicular to Forward, rotated 90° CCW in the arm's vertical plane.

TCP world position derived from (radius, up_down, pitch):
    r_tcp = radius * cos(pitch) - up_down * sin(pitch)   [horizontal reach from base axis]
    z_tcp = d_base + radius * sin(pitch) + up_down * cos(pitch)  [world height]

Link lengths (from transform.py — definitive values)
-----------------------------------------------------
    d_base  = 0.094605 m   shoulder pivot height above ground
    L1      = 0.10048  m   upper arm      (joint2 → joint3)
    L2      = 0.100    m   forearm        (joint3 → joint4)
    L3      = 0.055    m   wrist segment  (joint4 → wrist joint)
    L_tool  = 0.115    m   gripper        (wrist joint → TCP)
    L_tcp   = 0.170    m   joint4 → TCP   (= L3 + L_tool)

URDF ↔ arm-plane angle conventions
------------------------------------
    q2_std  = upper arm angle from horizontal  →  j2 = q2_std − π/2
    q3_std  = standard elbow relative angle    →  j3 = −q3_std
    j4      = pitch − π/2 + j3 − j2           (ensures cumulative pitch = desired)

Servo pulse order (angle2pulse index → servo ID):
    index 0 → S6  base rotation   (joint1)
    index 1 → S5  shoulder        (joint2)
    index 2 → S4  elbow           (joint3)
    index 3 → S3  wrist pitch     (joint4)
    index 4 → S2  wrist roll      (wrist)
    S1 (gripper) handled separately.
"""

import math
from dataclasses import dataclass
from typing import Optional

from .transform import (
    _map,
    joint1_map, joint2_map, joint3_map, joint4_map, joint5_map,
    clamp_pulses,
)

# ── Link lengths ──────────────────────────────────────────────────────────────
D_BASE = 0.094605
L1     = 0.10048
L2     = 0.100
L3     = 0.055
L_TOOL = 0.115
L_TCP  = L3 + L_TOOL   # 0.170 m  joint4 → TCP

# Gripper pulse range (servo 1, not part of IK chain)
_GRIPPER_OPEN   = 200
_GRIPPER_CLOSED = 680

# ── Floor guard ───────────────────────────────────────────────────────────────
# Minimum TCP height above the table surface (z = 0 is the ground plane).
# Keeps the gripper from driving into the table during telop and ACT inference.
# Lower this value if your task requires picking objects flush with the surface.
Z_TCP_MIN = 0.005   # 5 mm above table


# ── Result type ───────────────────────────────────────────────────────────────
@dataclass
class IKResult:
    """Output of a successful IK solve.

    joints : URDF joint angles in radians, keyed by joint name.
    pulses : servo pulse values, keyed by servo ID (1–6).
    reachable : True if the target was within workspace.
    """
    joints:    dict          # {'joint1': rad, 'joint2': rad, ..., 'wrist': rad}
    pulses:    dict          # {6: int, 5: int, 4: int, 3: int, 2: int, 1: int}
    reachable: bool = True


# ── Internal helpers ──────────────────────────────────────────────────────────

def _joint_angles_to_pulses(j1, j2, j3, j4, j5, gripper_fraction=0.0):
    """Convert URDF joint angles (radians) to servo pulse values.

    Inverse of the pulse→angle transforms in verify_fk.py:
        j1 =  radians(_map(p6, joint1_map))
        j2 = -radians(_map(p5, joint2_map)) - pi/2
        j3 =  radians(_map(p4, joint3_map))
        j4 = -radians(_map(p3, joint4_map)) - pi/2
        j5 =  radians(_map(p2, joint5_map))

    Returns dict keyed by servo ID.
    """
    deg = math.degrees

    p6 = _map(deg(j1),             joint1_map, inverse=True)
    p5 = _map(-(deg(j2) + 90.0),   joint2_map, inverse=True)
    p4 = _map(deg(j3),             joint3_map, inverse=True)
    p3 = _map(-(deg(j4) + 90.0),   joint4_map, inverse=True)
    p2 = _map(deg(j5),             joint5_map, inverse=True)

    # Gripper: 0.0 = open (pulse 200), 1.0 = closed (pulse 680)
    gripper_fraction = max(0.0, min(1.0, gripper_fraction))
    p1 = _GRIPPER_OPEN + gripper_fraction * (_GRIPPER_CLOSED - _GRIPPER_OPEN)

    raw = [p6, p5, p4, p3, p2, p1]
    clamped = clamp_pulses(raw)

    return {6: clamped[0], 5: clamped[1], 4: clamped[2],
            3: clamped[3], 2: clamped[4], 1: clamped[5]}


def _check_limits(j2, j3, j4):
    """Return True if joint angles are within safe servo limits.

    Limits derived from transform.py servo angle ranges.
    j2 URDF ∈ [−π/2,  π/2]   (servo angle ∈ [−180.2,  0.2] deg)
    j3 URDF ∈ [−2.10, 2.10]  (servo angle ∈ [−120.2, 120.2] deg)
    j4 URDF ∈ [−1.92, 1.92]  (servo angle ∈ [−200.2,  20.2] deg translated)
    """
    ok = True
    ok &= math.radians(-180.2) <= -(j2 + math.pi/2) <= math.radians(0.2)
    ok &= math.radians(-120.2) <= j3 <= math.radians(120.2)
    ok &= math.radians(-200.2) <= -(j4 + math.pi/2) <= math.radians(20.2)
    return ok


# ── Public API ────────────────────────────────────────────────────────────────

def solve(
    theta:         float,
    pitch:         float,
    radius:        float,
    up_down:       float,
    gripper_tilt:  float = 0.0,
    gripper:       float = 0.0,
    elbow_up:      bool  = False,
) -> IKResult:
    """Solve inverse kinematics for the ArmPi Ultra.

    Parameters
    ----------
    theta        : Base yaw in radians.  0 = forward, positive = CCW.
    pitch        : Camera/tool pitch from horizontal in radians.
                   0 = horizontal, π/2 = pointing straight up, negative = down.
    radius       : Distance along the camera optical axis from the shoulder
                   pivot to the TCP (metres).  Positive = forward.
    up_down      : Offset perpendicular to the camera axis (metres).
                   Positive = upward in camera frame.
    gripper_tilt : Wrist roll angle in radians.  0 = neutral.
    gripper      : Gripper fraction: 0.0 = fully open, 1.0 = fully closed.
    elbow_up     : True = elbow-up solution (default), False = elbow-down.

    Returns
    -------
    IKResult with .joints (URDF angles), .pulses (servo IDs), .reachable flag.
    If the target is unreachable, .reachable is False and angles/pulses reflect
    the nearest reachable configuration (clamped).
    """
    # ── Step 1: TCP world position from camera-frame inputs ───────────────────
    # Origin at shoulder pivot (r=0, z=d_base).
    # Camera forward = pitch direction; camera up = 90° CCW from forward.
    r_tcp = radius  * math.cos(pitch) - up_down * math.sin(pitch)
    z_tcp = D_BASE  + radius * math.sin(pitch) + up_down * math.cos(pitch)

    # ── Floor guard ───────────────────────────────────────────────────────────
    if z_tcp < Z_TCP_MIN:
        return IKResult(joints={}, pulses={}, reachable=False)

    # ── Step 2: Joint4 (wrist pivot) position in arm plane ───────────────────
    # TCP = joint4 + L_tcp along the tool direction.
    r_j4     = r_tcp - L_TCP * math.cos(pitch)
    z_j4_rel = z_tcp - D_BASE - L_TCP * math.sin(pitch)   # relative to shoulder

    # ── Step 3: 2R planar IK for joints 2 & 3 ────────────────────────────────
    D_sq = r_j4**2 + z_j4_rel**2
    D    = math.sqrt(D_sq)

    cos_q3 = (D_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
    reachable = True

    if cos_q3 < -1.0 or cos_q3 > 1.0:
        reachable = False
        cos_q3 = max(-1.0, min(1.0, cos_q3))

    q3_std = math.acos(cos_q3)                  # elbow-up: positive
    if not elbow_up:
        q3_std = -q3_std                         # elbow-down: negative

    gamma = math.atan2(z_j4_rel, r_j4)
    delta = math.atan2(L2 * math.sin(q3_std), L1 + L2 * math.cos(q3_std))
    q2_std = gamma - delta

    # ── Step 4: Convert arm-plane angles to URDF joint angles ─────────────────
    j1 = theta
    j2 = q2_std - math.pi / 2
    j3 = -q3_std
    j4 = pitch - math.pi / 2 + j3 - j2   # ensures cumulative pitch == desired
    j5 = gripper_tilt                     # wrist roll is independent

    # ── Step 5: Joint limit check ─────────────────────────────────────────────
    if not _check_limits(j2, j3, j4):
        reachable = False

    # ── Step 6: Build outputs ─────────────────────────────────────────────────
    joints = {
        'joint1':       j1,
        'joint2':       j2,
        'joint3':       j3,
        'joint4':       j4,
        'wrist':        j5,
        'gripper_joint': gripper * 0.785,
    }

    pulses = _joint_angles_to_pulses(j1, j2, j3, j4, j5, gripper)

    return IKResult(joints=joints, pulses=pulses, reachable=reachable)


def tcp_world_position(pitch, radius, up_down):
    """Return the world-frame (r, z) position of the TCP given camera-frame inputs.

    Useful for visualising or validating the workspace without running a full solve.
    r = horizontal reach from base axis; z = height above ground.
    """
    r = radius  * math.cos(pitch) - up_down * math.sin(pitch)
    z = D_BASE  + radius * math.sin(pitch) + up_down * math.cos(pitch)
    return r, z