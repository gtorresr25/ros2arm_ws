#!/usr/bin/env python3
"""
ik_viz_tcp.py — 2D stick figure IK debugger, TCP-pitch mode.

Sliders control the TCP position in world coordinates (r_tcp, z_tcp).
Pitch rotates the tool AT the TCP — the tip stays fixed, arm reconfigures.

Usage:
    python3 scripts/ik_viz_tcp.py
"""

import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider

# ── Link lengths (from transform.py) ─────────────────────────────────────────
D_BASE = 0.094605
L1     = 0.10048
L2     = 0.100
L3     = 0.055
L_TOOL = 0.115
L_TCP  = L3 + L_TOOL  # 0.170

# ── Saved home positions (from README) ───────────────────────────────────────
HOMES = {
    'home1': {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230},
    'home2': {6: 504, 5: 510, 4: 496, 3: 502, 2: 499, 1: 230},
    'home3': {6: 504, 5: 716, 4: 829, 3: 260, 2: 498, 1: 230},
    'home4': {6: 504, 5: 597, 4: 749, 3:  82, 2: 498, 1: 230},
}

_J2_MAP = [0, 1000, 500,   30, -210, -90]
_J3_MAP = [0, 1000, 500, -120,  120,   0]
_J4_MAP = [0, 1000, 500,   30, -210, -90]

def _map_fwd(value, param):
    return ((value - param[2]) / (param[1] - param[0])) * (param[4] - param[3]) + param[5]

def home_to_sliders(home_name):
    """Convert a saved home position to (pitch_deg, r_tcp, z_tcp) slider values."""
    p = HOMES[home_name]
    j2 = -math.radians(_map_fwd(p[5], _J2_MAP)) - math.pi / 2
    j3 =  math.radians(_map_fwd(p[4], _J3_MAP))
    j4 = -math.radians(_map_fwd(p[3], _J4_MAP)) - math.pi / 2

    q2_std = j2 + math.pi / 2
    q3_std = -j3
    pitch  = j4 + math.pi / 2 - j3 + j2

    elbow_r = L1 * math.cos(q2_std)
    elbow_z = D_BASE + L1 * math.sin(q2_std)
    fa      = q2_std + q3_std
    wrist_r = elbow_r + L2 * math.cos(fa)
    wrist_z = elbow_z + L2 * math.sin(fa)
    tcp_r   = wrist_r + L_TCP * math.cos(pitch)
    tcp_z   = wrist_z + L_TCP * math.sin(pitch)

    return math.degrees(pitch), tcp_r, tcp_z

# ── IK solver (inline, no imports) ───────────────────────────────────────────

def solve(pitch, radius, up_down, elbow_up=False):
    """Returns (shoulder, elbow, wrist, tcp) as (r, z) tuples, plus reachable flag."""
    shoulder = (0.0, D_BASE)

    r_tcp = radius  * math.cos(pitch) - up_down * math.sin(pitch)
    z_tcp = D_BASE  + radius * math.sin(pitch) + up_down * math.cos(pitch)

    r_j4     = r_tcp - L_TCP * math.cos(pitch)
    z_j4_rel = z_tcp - D_BASE - L_TCP * math.sin(pitch)

    D_sq   = r_j4**2 + z_j4_rel**2
    cos_q3 = (D_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
    reachable = -1.0 <= cos_q3 <= 1.0
    cos_q3 = max(-1.0, min(1.0, cos_q3))

    q3_std = math.acos(cos_q3) * (1 if elbow_up else -1)
    gamma  = math.atan2(z_j4_rel, r_j4)
    delta  = math.atan2(L2 * math.sin(q3_std), L1 + L2 * math.cos(q3_std))
    q2_std = gamma - delta

    elbow = (L1 * math.cos(q2_std),
             D_BASE + L1 * math.sin(q2_std))

    forearm_angle = q2_std + q3_std
    wrist = (elbow[0] + L2 * math.cos(forearm_angle),
             elbow[1] + L2 * math.sin(forearm_angle))

    tcp = (r_tcp, z_tcp)

    return shoulder, elbow, wrist, tcp, reachable

def _world_to_cam(pitch, r_tcp, z_tcp):
    z_rel   = z_tcp - D_BASE
    radius  =  r_tcp * math.cos(pitch) + z_rel * math.sin(pitch)
    up_down = -r_tcp * math.sin(pitch) + z_rel * math.cos(pitch)
    return radius, up_down

# ── Plot setup ────────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(7, 8))
plt.subplots_adjust(bottom=0.30)
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.1, 0.6)
ax.set_aspect('equal')
ax.set_xlabel('r  (metres, horizontal reach)')
ax.set_ylabel('z  (metres, height)')
ax.set_title('ArmPi Ultra — 2D IK  [TCP-pitch mode]')
ax.axhline(0, color='brown', linewidth=2, label='ground')
ax.axhline(D_BASE, color='grey', linestyle='--', linewidth=0.8, label=f'shoulder z={D_BASE:.3f}m')
ax.grid(True, alpha=0.3)

_p, _r, _z     = home_to_sliders('home1')
INIT_PITCH_DEG = _p
INIT_R_TCP     = _r
INIT_Z_TCP     = _z

_init_pitch = math.radians(INIT_PITCH_DEG)
_init_rad, _init_ud = _world_to_cam(_init_pitch, INIT_R_TCP, INIT_Z_TCP)
shoulder, elbow, wrist, tcp, ok = solve(_init_pitch, _init_rad, _init_ud)

arm_line,  = ax.plot([shoulder[0], elbow[0], wrist[0], tcp[0]],
                     [shoulder[1], elbow[1], wrist[1], tcp[1]],
                     'b-o', linewidth=2, markersize=8, label='arm links')

tcp_dot,   = ax.plot(*tcp, 'r*', markersize=14, label='TCP (gripper tip)')

pitch_line, = ax.plot(
    [0, 0.3 * math.cos(_init_pitch)],
    [D_BASE, D_BASE + 0.3 * math.sin(_init_pitch)],
    'g--', linewidth=1, alpha=0.6, label='camera axis'
)

status_text = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                      verticalalignment='top', fontsize=9,
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

ax.legend(loc='upper right', fontsize=8)

ax_pitch  = plt.axes([0.15, 0.20, 0.70, 0.03])
ax_rtcp   = plt.axes([0.15, 0.14, 0.70, 0.03])
ax_ztcp   = plt.axes([0.15, 0.08, 0.70, 0.03])

sl_pitch  = Slider(ax_pitch, 'pitch (deg)', -90, 90,   valinit=INIT_PITCH_DEG, valstep=1)
sl_rtcp   = Slider(ax_rtcp,  'r_tcp (m)',   0.0, 0.45, valinit=INIT_R_TCP,    valstep=0.005)
sl_ztcp   = Slider(ax_ztcp,  'z_tcp (m)',   0.0, 0.55, valinit=INIT_Z_TCP,    valstep=0.005)


def update(_):
    pitch  = math.radians(sl_pitch.val)
    r_tcp  = sl_rtcp.val
    z_tcp  = sl_ztcp.val

    radius, up_down = _world_to_cam(pitch, r_tcp, z_tcp)
    shoulder, elbow, wrist, tcp, reachable = solve(pitch, radius, up_down)

    xs = [shoulder[0], elbow[0], wrist[0], tcp[0]]
    zs = [shoulder[1], elbow[1], wrist[1], tcp[1]]
    arm_line.set_xdata(xs)
    arm_line.set_ydata(zs)
    tcp_dot.set_xdata([tcp[0]])
    tcp_dot.set_ydata([tcp[1]])

    pitch_line.set_xdata([0, 0.3 * math.cos(pitch)])
    pitch_line.set_ydata([D_BASE, D_BASE + 0.3 * math.sin(pitch)])

    q2_std = math.atan2(elbow[1] - D_BASE, elbow[0])
    q3_vec = (wrist[0] - elbow[0], wrist[1] - elbow[1])
    forearm_angle = math.atan2(q3_vec[1], q3_vec[0])
    q3_std = forearm_angle - q2_std
    j2 = q2_std - math.pi / 2
    j3 = -q3_std
    j4 = pitch - math.pi / 2 + j3 - j2

    color = 'wheat' if reachable else 'salmon'
    status_text.get_bbox_patch().set_facecolor(color)
    status_text.set_text(
        f"{'REACHABLE' if reachable else 'OUT OF REACH'}\n"
        f"TCP  r={tcp[0]*100:.1f}cm  z={tcp[1]*100:.1f}cm\n"
        f"j2={math.degrees(j2):+.1f}°  j3={math.degrees(j3):+.1f}°  j4={math.degrees(j4):+.1f}°"
    )
    fig.canvas.draw_idle()


sl_pitch.on_changed(update)
sl_rtcp.on_changed(update)
sl_ztcp.on_changed(update)
update(None)

plt.show()