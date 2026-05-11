"""
ik.py — ArmPi Ultra inverse and forward kinematics.

Uses ikpy + the URDF for geometry, and transform.py for servo pulse mapping.

Coordinate convention
---------------------
  Origin : servo 6 rotation axis (arm base plate center)
  X      : forward
  Y      : left
  Z      : up
  Units  : metres

Public API
----------
  solve(x, y, z, current_pulses=None)  →  {servo_id: pulse} or None
  fk(pulses)                           →  (x, y, z) in metres

Sign conventions (from physical joint observation with joint_jog.py)
--------------------------------------------------------------------
  Shoulder (S5, ikpy joint2):    same sign as URDF,  offset −90°
  Elbow    (S4, ikpy joint3):    inverted vs URDF,   offset   0°
  Wrist pitch (S3, ikpy joint4): same sign as URDF,  offset −90°
  Wrist roll  (S2, ikpy joint5): inverted vs URDF,   offset   0°

  servo_angle(S5) =  ikpy_j2_deg − 90
  servo_angle(S4) = −ikpy_j3_deg
  servo_angle(S3) =  ikpy_j4_deg − 90
  servo_angle(S2) = −ikpy_j5_deg

  ikpy_j2 = radians(servo_S5_deg + 90)
  ikpy_j3 = radians(−servo_S4_deg)
  ikpy_j4 = radians(servo_S3_deg + 90)
  ikpy_j5 = radians(−servo_S2_deg)
"""

import os
import sys
import math
import warnings
import numpy as np

# Suppress ikpy warnings about fixed joints with axis attributes
warnings.filterwarnings('ignore', category=UserWarning, module='ikpy')

# ikpy installed as user package
_IKPY_PATH = os.path.expanduser('~/.local/lib/python3.12/site-packages')
if _IKPY_PATH not in sys.path:
    sys.path.insert(0, _IKPY_PATH)

from ikpy.chain import Chain
from ament_index_python.packages import get_package_share_directory

from .transform import (
    _map,
    joint1_map, joint2_map, joint3_map, joint4_map, joint5_map,
    clamp_pulses,
)

# ── Constants ─────────────────────────────────────────────────────────────────

# Extra distance from ikpy chain tip to jaw center along arm axis.
# Calibrated: home2 measured z=330mm, ikpy gives ~295mm → offset=35mm.
_EE_OFFSET = 0.035   # metres

# Home2 servo pulses (arm straight up) — used as default IK initial guess
_HOME2_PULSES = {6: 504, 5: 510, 4: 496, 3: 502, 2: 499, 1: 230}

# ── Load ikpy chain ───────────────────────────────────────────────────────────

# Single source of truth: the armpi_ultra_description URDF
_URDF = os.path.join(
    get_package_share_directory('armpi_ultra_description'),
    'urdf', 'armpi_ultra.urdf'
)

_chain = Chain.from_urdf_file(
    _URDF,
    active_links_mask=[False, False, True, True, True, True, False, False],
)

# ── Internal helpers ──────────────────────────────────────────────────────────

def _pulses_to_ikpy(pulses: dict) -> list:
    """Convert servo pulse dict to 8-element ikpy angle vector."""
    s5 = _map(pulses.get(5, _HOME2_PULSES[5]), joint2_map)
    s4 = _map(pulses.get(4, _HOME2_PULSES[4]), joint3_map)
    s3 = _map(pulses.get(3, _HOME2_PULSES[3]), joint4_map)
    s2 = _map(pulses.get(2, _HOME2_PULSES[2]), joint5_map)

    j2 = math.radians( s5 + 90)
    j3 = math.radians(-s4)
    j4 = math.radians( s3 + 90)
    j5 = math.radians(-s2)

    return [0.0, 0.0, j2, j3, j4, j5, 0.0, 0.0]


def _ikpy_to_pulses(angles: list, s6_pulse: int) -> dict:
    """Convert 8-element ikpy angle vector + base pulse to servo pulse dict."""
    j2 = math.degrees(angles[2])
    j3 = math.degrees(angles[3])
    j4 = math.degrees(angles[4])
    j5 = math.degrees(angles[5])

    s5_angle =  j2 - 90
    s4_angle = -j3
    s3_angle =  j4 - 90
    s2_angle = -j5

    raw = [
        _map(s5_angle, joint2_map, inverse=True),
        _map(s4_angle, joint3_map, inverse=True),
        _map(s3_angle, joint4_map, inverse=True),
        _map(s2_angle, joint5_map, inverse=True),
    ]
    p5, p4, p3, p2 = clamp_pulses(raw)

    return {6: s6_pulse, 5: p5, 4: p4, 3: p3, 2: p2}


# ── Public API ────────────────────────────────────────────────────────────────

def solve(x: float, y: float, z: float,
          current_pulses: dict = None) -> dict | None:
    """
    Inverse kinematics.

    Parameters
    ----------
    x, y, z        : target jaw-center position (metres) relative to servo 6
    current_pulses : {servo_id: pulse} used as warm-start initial guess.
                     Defaults to home2 if not provided.

    Returns
    -------
    {servo_id: pulse} for servos 2-6, or None if ikpy found no solution.
    """
    # Base rotation — servo 6
    s6_angle = math.degrees(math.atan2(y, x))
    s6_pulse  = clamp_pulses([_map(s6_angle, joint1_map, inverse=True)])[0]

    # Radial reach in the arm's plane
    r = math.sqrt(x**2 + y**2)

    # IK target in URDF frame.
    # Subtract EE_OFFSET from z — approximation that treats the arm as
    # roughly vertical; fine for most poses, revisit if large pitch angles
    # cause visible error.
    target_urdf = _BASE_OFFSET + np.array([r, 0.0, z - _EE_OFFSET])

    initial = _pulses_to_ikpy(current_pulses if current_pulses else _HOME2_PULSES)

    try:
        result = _chain.inverse_kinematics(
            target_position=target_urdf,
            initial_position=initial,
        )
    except Exception:
        return None

    return _ikpy_to_pulses(result, s6_pulse)


def fk(pulses: dict) -> tuple[float, float, float]:
    """
    Forward kinematics.

    Parameters
    ----------
    pulses : {servo_id: pulse}

    Returns
    -------
    (x, y, z) jaw-center position in metres relative to servo 6.
    """
    # Base rotation angle
    s6_deg  = _map(pulses.get(6, _HOME2_PULSES[6]), joint1_map)
    theta1  = math.radians(s6_deg)

    # Run ikpy FK in URDF frame
    mat     = _chain.forward_kinematics(_pulses_to_ikpy(pulses))
    tip_urdf = mat[:3, 3]

    # Position relative to servo 6 base
    tip_rel = tip_urdf - _BASE_OFFSET

    # Add EE offset along the end-effector's Z axis (arm pointing direction)
    ee_z_axis = mat[:3, 2]
    tip_rel   = tip_rel + _EE_OFFSET * ee_z_axis

    # Rotate from arm plane (URDF frame) to world frame via base rotation
    c = math.cos(theta1)
    s = math.sin(theta1)
    x_w =  c * tip_rel[0] - s * tip_rel[1]
    y_w =  s * tip_rel[0] + c * tip_rel[1]
    z_w =  tip_rel[2]

    return x_w, y_w, z_w
