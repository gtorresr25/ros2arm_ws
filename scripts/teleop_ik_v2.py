#!/usr/bin/env python3
"""
teleop_ik_v2.py — Mouse + Keyboard teleop for ArmPi Ultra.

Mouse controls
--------------
  Move left / right   theta     base rotation
  Move up / down      up_down   vertical position
  Scroll up           radius    retract (inward)
  Scroll down         radius    extend (outward)
  Left click          close gripper a step
  Right click         open gripper a step

Keyboard controls
-----------------
  Q / E    pitch       tilt up / tilt down
  A / D    tilt        wrist roll left / right
  I / O    recording   start / stop episode recording
  H        home1
  0        mouse off (re-enable with 1–9)
  1 – 9    sensitivity multiplier
  Esc / Ctrl-C   quit

Requires:
  colcon build --packages-select kinematics
  source install/setup.bash
"""

import sys
import os
import math
import time
import tty
import termios
import select
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool

# ── SDK ───────────────────────────────────────────────────────────────────────
from ros_robot_controller_sdk import Board

# ── IK ────────────────────────────────────────────────────────────────────────
from kinematics.ik import solve, D_BASE, L1, L2, L_TCP
from kinematics.transform import _map, joint1_map, joint2_map, joint3_map, joint4_map, joint5_map

# ── Hardware ──────────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 1_000_000
MOVE_DURATION = 0.08    # servo travel time per arm command

# ── Command rate ──────────────────────────────────────────────────────────────
CMD_HZ       = 12
CMD_INTERVAL = 1.0 / CMD_HZ    # ~83 ms

# ── Smoothing ─────────────────────────────────────────────────────────────────
ALPHA = 0.2     # 1.0 = instant, 0.1 = very smooth/laggy

# ── Motion sensitivity ────────────────────────────────────────────────────────
SENS_BASE = {
    'theta':   math.radians(0.5),
    'up_down': 0.0010,
    'radius':  0.004,
    'pitch':   math.radians(3),
    'tilt':    math.radians(3),
}
THETA_LIMIT = math.radians(110)
TILT_LIMIT  = math.radians(120)

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIP_CLICK_STEP = 0.1   # fraction change per click (0.0 = open, 1.0 = closed)

# ── Home ──────────────────────────────────────────────────────────────────────
HOME1_PULSES = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}

def pulses_to_joint_angles(p: dict) -> list:
    """p = {6: pulse, 5: pulse, ..., 1: pulse}
    Returns [j1, j2, j3, j4, wrist, gripper_rad] float32."""
    j1      =  math.radians(_map(p[6], joint1_map))
    j2      = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3      =  math.radians(_map(p[4], joint3_map))
    j4      = -math.radians(_map(p[3], joint4_map)) - math.pi / 2
    wrist   =  math.radians(_map(p[2], joint5_map))
    gripper = (p[1] - 200) / (680 - 200) * 0.785   # 200=open→0 rad, 680=closed→0.785 rad
    return [j1, j2, j3, j4, wrist, gripper]


def read_servo_pulses(board) -> dict:
    """Read back actual pulse positions from all 6 servos.
    Returns {6: pulse, 5: pulse, …, 1: pulse} or {} on failure."""
    pulses = {}
    try:
        for sid in (6, 5, 4, 3, 2, 1):
            result = board.bus_servo_read_position(sid)
            if result is None:
                return {}
            pulses[sid] = result[-1]   # <BBbh → last field is int16 pulse
    except Exception:
        return {}
    return pulses


def _lpf(des, smo):
    """First-order low-pass (exponential smoothing)."""
    return ALPHA * des + (1 - ALPHA) * smo

def pulses_to_state(p):
    theta = math.radians(_map(p[6], joint1_map))
    j2    = -math.radians(_map(p[5], joint2_map)) - math.pi / 2
    j3    =  math.radians(_map(p[4], joint3_map))
    j4    = -math.radians(_map(p[3], joint4_map)) - math.pi / 2

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

    z_rel   = tcp_z - D_BASE
    radius  =  tcp_r * math.cos(pitch) + z_rel * math.sin(pitch)
    up_down = -tcp_r * math.sin(pitch) + z_rel * math.cos(pitch)
    return theta, pitch, radius, up_down

# ── ROS2 publisher ────────────────────────────────────────────────────────────
class JointPublisher(Node):
    def __init__(self):
        super().__init__('teleop_ik_v2')
        self.pub     = self.create_publisher(JointState, '/joint_states', 10)
        self.ik_pub  = self.create_publisher(Float32MultiArray, '/teleop/ik_state', 10)
        self.ja_pub  = self.create_publisher(Float32MultiArray, '/teleop/joint_angles', 10)
        self.rec_pub = self.create_publisher(Bool, '/teleop/recording', 10)

    def publish(self, joints: dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(joints.keys())
        msg.position = list(joints.values())
        self.pub.publish(msg)

    def publish_ik_state(self, theta, pitch, radius, up_down, gripper_frac, tilt=0.0):
        # Field order: [theta, pitch, radius, up_down, gripper_frac, tilt]
        msg = Float32MultiArray()
        msg.data = [float(theta), float(pitch), float(radius),
                    float(up_down), float(gripper_frac), float(tilt)]
        self.ik_pub.publish(msg)

    def publish_joint_angles(self, angles: list):
        """Publish actual servo joint angles [j1,j2,j3,j4,wrist,gripper_rad]."""
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.ja_pub.publish(msg)

    def publish_recording(self, recording: bool):
        msg = Bool()
        msg.data = recording
        self.rec_pub.publish(msg)

# ── Terminal mouse ────────────────────────────────────────────────────────────
_MOUSE_ON  = '\x1b[?1000h\x1b[?1002h\x1b[?1003h\x1b[?1006h'
_MOUSE_OFF = '\x1b[?1000l\x1b[?1002l\x1b[?1003l\x1b[?1006l'

def _mouse_on():
    sys.stdout.write(_MOUSE_ON); sys.stdout.flush()

def _mouse_off():
    sys.stdout.write(_MOUSE_OFF); sys.stdout.flush()

# ── Input parser ──────────────────────────────────────────────────────────────
def read_event(fd, esc_timeout=0.05):
    ch = os.read(fd, 1).decode('latin-1')
    if ch != '\x1b':
        return ('key', ch)

    r, _, _ = select.select([fd], [], [], esc_timeout)
    if not r:
        return ('key', '\x1b')

    ch2 = os.read(fd, 1).decode('latin-1')
    if ch2 != '[':
        return ('key', '\x1b' + ch2)

    buf = ''
    while True:
        c = os.read(fd, 1).decode('latin-1')
        buf += c
        if c.isalpha() or c == '~':
            break

    if buf.startswith('<') and buf[-1] in ('M', 'm'):
        try:
            parts   = buf[1:-1].split(';')
            btn     = int(parts[0])
            col     = int(parts[1])
            row     = int(parts[2])
            pressed = (buf[-1] == 'M')
            return ('mouse', (btn, col, row, pressed))
        except (ValueError, IndexError):
            pass

    return ('key', '\x1b[' + buf)

# ── Display ───────────────────────────────────────────────────────────────────
def fmt_state(theta, pitch, radius, up_down, tilt, sens, reachable, note=''):
    tag      = 'OK' if reachable else '!!'
    sens_str = 'MOUSE OFF' if sens == 0 else f'sens×{sens}'
    line = (f"[{tag}]  "
            f"θ={math.degrees(theta):+6.1f}°  "
            f"pitch={math.degrees(pitch):+6.1f}°  "
            f"r={radius*100:5.1f}cm  "
            f"up={up_down*100:+5.1f}cm  "
            f"roll={math.degrees(tilt):+6.1f}°  "
            f"{sens_str}")
    return line + (f"  {note}" if note else '')

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    ros_node = JointPublisher()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

    print(f"[INFO] Connecting on {SERIAL_PORT} …")
    board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
    board.enable_reception(True)
    time.sleep(0.3)
    print("[OK]  Connected.")

    for sid in range(1, 7):
        board.bus_servo_enable_torque(sid, False)
        time.sleep(0.02)

    print("Moving to home1 …")
    board.bus_servo_set_position(4.0, [[sid, p] for sid, p in HOME1_PULSES.items()])
    time.sleep(4.5)

    theta, pitch, radius, up_down = pulses_to_state(HOME1_PULSES)
    gripper_frac_state = [0.0]   # shared with grip thread — index 0 is live frac
    recording          = False
    sens               = 1
    reachable          = True

    ros_node.publish(solve(theta, pitch, radius, up_down, gripper=0.0).joints)

    print("\nReady.")
    print("  Mouse move    → pan left/right (θ) and up/down")
    print("  Scroll        → radius in / out")
    print("  Left click    → grip     Right click → release")
    print("  Q / E         → pitch up / down")
    print("  A / D         → wrist roll left / right")
    print("  I → start recording     O → stop recording")
    print("  H → home1     0 → mouse off     1–9 → sensitivity     Esc → quit\n")
    print(fmt_state(theta, pitch, radius, up_down, 0.0, sens, reachable))

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    des_theta,  des_pitch,  des_radius,  des_up_down,  des_tilt  = theta, pitch, radius, up_down, 0.0
    smo_theta,  smo_pitch,  smo_radius,  smo_up_down,  smo_tilt  = theta, pitch, radius, up_down, 0.0
    sent_theta, sent_pitch, sent_radius, sent_up_down, sent_tilt = theta, pitch, radius, up_down, 0.0

    last_cmd_t = time.monotonic()
    last_mouse = None
    note       = ''

    try:
        tty.setraw(fd)
        _mouse_on()

        while True:
            # ── Wait for input or next command tick ───────────────────────────
            now  = time.monotonic()
            wait = max(0.0, CMD_INTERVAL - (now - last_cmd_t))
            r, _, _ = select.select([fd], [], [], wait)

            # ── Drain all buffered input events ───────────────────────────────
            if r:
                while True:
                    etype, edata = read_event(fd)

                    if etype == 'key':
                        key = edata

                        if key in ('\x1b', '\x03'):
                            raise KeyboardInterrupt

                        elif key in '0123456789':
                            sens = int(key)

                        elif key.lower() == 'h':
                            _mouse_off()
                            board.bus_servo_set_position(
                                2.0, [[sid, p] for sid, p in HOME1_PULSES.items()
                                      if sid != 1])
                            time.sleep(2.5)
                            t0, p0, r0, u0 = pulses_to_state(HOME1_PULSES)
                            des_theta,  des_pitch,  des_radius,  des_up_down,  des_tilt  = t0, p0, r0, u0, 0.0
                            smo_theta,  smo_pitch,  smo_radius,  smo_up_down,  smo_tilt  = t0, p0, r0, u0, 0.0
                            sent_theta, sent_pitch, sent_radius, sent_up_down, sent_tilt = t0, p0, r0, u0, 0.0
                            ros_node.publish(solve(t0, p0, r0, u0,
                                                   gripper=gripper_frac_state[0]).joints)
                            last_mouse = None
                            last_cmd_t = time.monotonic()
                            reachable  = True
                            note       = '[home1]'
                            _mouse_on()

                        elif key.lower() == 'q':
                            des_pitch += sens * SENS_BASE['pitch']

                        elif key.lower() == 'e':
                            des_pitch -= sens * SENS_BASE['pitch']

                        elif key.lower() == 'a':
                            des_tilt = max(-TILT_LIMIT, des_tilt - sens * SENS_BASE['tilt'])

                        elif key.lower() == 'd':
                            des_tilt = min(TILT_LIMIT, des_tilt + sens * SENS_BASE['tilt'])

                        elif key.lower() == 'i':
                            recording = True
                            note = '[REC]'

                        elif key.lower() == 'o':
                            recording = False
                            note = '[STOP]'

                    elif etype == 'mouse':
                        btn, col, row, pressed = edata

                        if btn in (32, 33, 34, 35):             # motion
                            if last_mouse is not None:
                                dcol = col - last_mouse[0]
                                drow = row - last_mouse[1]
                                des_theta   -= dcol * sens * SENS_BASE['theta']
                                des_theta    = max(-THETA_LIMIT, min(THETA_LIMIT, des_theta))
                                des_up_down -= drow * sens * SENS_BASE['up_down']
                            last_mouse = (col, row)

                        elif btn == 64:                          # scroll up → retract
                            des_radius -= sens * SENS_BASE['radius']

                        elif btn == 65:                          # scroll down → extend
                            des_radius += sens * SENS_BASE['radius']

                        elif btn == 0 and pressed and sens != 0:   # left click → close
                            gripper_frac_state[0] = min(1.0, gripper_frac_state[0] + GRIP_CLICK_STEP)

                        elif btn == 2 and pressed and sens != 0:   # right click → open
                            gripper_frac_state[0] = max(0.0, gripper_frac_state[0] - GRIP_CLICK_STEP)

                    more, _, _ = select.select([fd], [], [], 0)
                    if not more:
                        break

            # ── Timed arm command (12 Hz) ─────────────────────────────────────
            now = time.monotonic()
            if now - last_cmd_t >= CMD_INTERVAL:

                smo_theta   = _lpf(des_theta,   smo_theta)
                smo_pitch   = _lpf(des_pitch,   smo_pitch)
                smo_radius  = _lpf(des_radius,  smo_radius)
                smo_up_down = _lpf(des_up_down, smo_up_down)
                smo_tilt    = _lpf(des_tilt,    smo_tilt)

                result    = solve(smo_theta, smo_pitch, smo_radius, smo_up_down,
                                  gripper_tilt=smo_tilt, gripper=gripper_frac_state[0])
                reachable = result.reachable

                if reachable:
                    p = result.pulses
                    board.bus_servo_set_position(
                        MOVE_DURATION,
                        [[6, p[6]], [5, p[5]], [4, p[4]], [3, p[3]], [2, p[2]], [1, p[1]]])

                    # Read back actual positions; fall back to commanded pulses on failure
                    readback = read_servo_pulses(board)
                    joint_angles = pulses_to_joint_angles(readback if readback else p)

                    ros_node.publish(result.joints)
                    ros_node.publish_ik_state(smo_theta, smo_pitch, smo_radius,
                                              smo_up_down, gripper_frac_state[0], smo_tilt)
                    ros_node.publish_joint_angles(joint_angles)
                    ros_node.publish_recording(recording)
                    sent_theta, sent_pitch, sent_radius, sent_up_down, sent_tilt = (
                        smo_theta, smo_pitch, smo_radius, smo_up_down, smo_tilt)
                    if note == '[home1]':
                        note = ''
                else:
                    des_theta,  des_pitch,  des_radius,  des_up_down,  des_tilt  = (
                        sent_theta, sent_pitch, sent_radius, sent_up_down, sent_tilt)
                    smo_theta,  smo_pitch,  smo_radius,  smo_up_down,  smo_tilt  = (
                        sent_theta, sent_pitch, sent_radius, sent_up_down, sent_tilt)

                last_cmd_t = now

                print(f"\r{fmt_state(smo_theta, smo_pitch, smo_radius, smo_up_down, smo_tilt, sens, reachable, note)}   ",
                      end='', flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        _mouse_off()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n[INFO] Quit.")
        board.port.close()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
