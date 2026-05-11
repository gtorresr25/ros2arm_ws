"""
transform.py — ArmPi Ultra servo ↔ angle conversions.

Extracted from the vendor reference (driver/kinematics/kinematics/transform.py)
and cleaned for Python 3.12 compatibility:
  - Removed geometry_msgs imports (only needed for rot2qua / qua2rpy, which we
    don't use for IK/FK).
  - Removed CHASSIS_TYPE environment dependency; base_link is fixed to the
    ArmPi Ultra standard value (not the Slide_Rails variant).

Servo ↔ IK index mapping
--------------------------
angle2pulse / pulse2angle work on a 5-element vector ordered as follows:

  index 0  →  servo 6  (base rotation)
  index 1  →  servo 5  (shoulder)
  index 2  →  servo 4  (elbow)
  index 3  →  servo 3  (wrist pitch)
  index 4  →  servo 2  (wrist rotation)

Servo 1 (gripper) is NOT part of the IK chain.
"""

import numpy as np
from math import degrees, radians, atan2, asin, sqrt

# ── Link lengths (metres) ─────────────────────────────────────────────────────
base_link  = 0.094605   # height from ground to shoulder pivot
link1      = 0.10048    # upper arm
link2      = 0.100      # forearm
link3      = 0.055      # wrist segment
tool_link  = 0.115      # gripper length

# ── Joint limits (degrees) ───────────────────────────────────────────────────
joint1 = [-120.2,  120.2]   # base rotation
joint2 = [-180.2,    0.2]   # shoulder
joint3 = [-120.2,  120.2]   # elbow
joint4 = [-200.2,   20.2]   # wrist pitch
joint5 = [-120.2,  120.2]   # wrist rotation

# ── Servo pulse ↔ angle maps ──────────────────────────────────────────────────
# Format: [pulse_min, pulse_max, pulse_center, angle_at_min, angle_at_max, angle_at_center]
joint1_map = [0, 1000, 500, -120,  120,   0]
joint2_map = [0, 1000, 500,   30, -210, -90]
joint3_map = [0, 1000, 500, -120,  120,   0]
joint4_map = [0, 1000, 500,   30, -210, -90]
joint5_map = [0, 1000, 500, -120,  120,   0]


def _map(value, param, inverse=False):
    """Linear interpolation between two calibrated points.

    Forward  (inverse=False): pulse  → angle (degrees)
    Inverse  (inverse=True):  angle  → pulse
    """
    if inverse:
        return ((value - param[5]) / (param[4] - param[3])) * (param[1] - param[0]) + param[2]
    else:
        return ((value - param[2]) / (param[1] - param[0])) * (param[4] - param[3]) + param[5]


def pulse2angle(pulses):
    """Convert 5 servo pulse values to joint angles (radians).

    pulses: sequence of 5 pulse values [p0, p1, p2, p3, p4]
            ordered by IK index (servo 6, 5, 4, 3, 2).
    Returns: tuple of 5 angles in radians.
    """
    return (
        radians(_map(pulses[0], joint1_map)),
        radians(_map(pulses[1], joint2_map)),
        radians(_map(pulses[2], joint3_map)),
        radians(_map(pulses[3], joint4_map)),
        radians(_map(pulses[4], joint5_map)),
    )


def angle2pulse(solutions, convert_int=False):
    """Convert IK angle solutions to servo pulse values.

    solutions: list of 5-element angle arrays (radians), one per IK solution.
    Returns:   list of 5-element pulse arrays, one per solution.
               Pulse order matches IK index: [servo6, servo5, servo4, servo3, servo2].
    """
    result = []
    for angles in solutions:
        p = [
            _map(degrees(angles[0]), joint1_map, inverse=True),
            _map(degrees(angles[1]), joint2_map, inverse=True),
            _map(degrees(angles[2]), joint3_map, inverse=True),
            _map(degrees(angles[3]), joint4_map, inverse=True),
            _map(degrees(angles[4]), joint5_map, inverse=True),
        ]
        if convert_int:
            p = [int(v) for v in p]
        result.append(p)
    return result


def clamp_pulses(pulses, lo=0, hi=1000):
    """Clamp each pulse value to [lo, hi]."""
    return [max(lo, min(hi, int(round(v)))) for v in pulses]