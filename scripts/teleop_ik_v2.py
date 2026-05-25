#!/usr/bin/env python3
"""
teleop_ik_v2.py — Mouse + Keyboard teleop for ArmPi Ultra.

Mouse controls
--------------
  Move left / right   theta     base rotation
  Move up / down      up_down   vertical position
  Scroll up           radius    retract (inward)
  Scroll down         radius    extend (outward)
  Left click          grip (stall-detected, runs in background)
  Right click         release gripper

Keyboard controls
-----------------
  Q / E    pitch       tilt up / tilt down
  H        home1
  0        mouse off (re-enable with 1–9)
  1 – 9    sensitivity multiplier
  P        print current state
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

# ── SDK ───────────────────────────────────────────────────────────────────────
SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)
from ros_robot_controller_sdk import Board

# ── IK ────────────────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/andres/ros2arm_ws/install/kinematics/lib/python3/dist-packages")
from kinematics.ik import solve, D_BASE, L1, L2, L_TCP

# ── Hardware ──────────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 1_000_000
MOVE_DURATION = 0.08    # servo travel time per arm command

# ── Command rate ──────────────────────────────────────────────────────────────
CMD_HZ       = 12
CMD_INTERVAL = 1.0 / CMD_HZ    # ~83 ms

# ── Smoothing ─────────────────────────────────────────────────────────────────
ALPHA = 0.4     # 1.0 = instant, 0.1 = very smooth/laggy

# ── Motion sensitivity ────────────────────────────────────────────────────────
SENS_BASE = {
    'theta':   math.radians(0.5),
    'up_down': 0.0010,
    'radius':  0.004,
    'pitch':   math.radians(3),
}
THETA_LIMIT = math.radians(110)

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIP_OPEN     = 200     # fully open pulse
GRIP_MAX      = 610     # max safe closing pulse
GRIP_STEP     = 10      # pulse increment per step
GRIP_STALL_TH = 40      # (commanded − actual) threshold → contact detected
GRIP_STEP_DT  = 0.05    # seconds between steps

def grip_until_stall(board, lock, stop_flag):
    """
    Open gripper then close incrementally until stall (contact) or max pulse.
    Runs in a background thread. lock serializes serial port with arm commands.
    stop_flag: threading.Event — set it to abort mid-grip (e.g. on release).
    """
    with lock:
        board.bus_servo_set_position(0.6, [[1, GRIP_OPEN]])
    time.sleep(0.7)

    pulse = GRIP_OPEN
    while pulse < GRIP_MAX and not stop_flag.is_set():
        pulse = min(pulse + GRIP_STEP, GRIP_MAX)
        with lock:
            board.bus_servo_set_position(0.05, [[1, pulse]])
        time.sleep(GRIP_STEP_DT)
        with lock:
            raw = board.bus_servo_read_position(1)
        actual = raw[0] if isinstance(raw, (list, tuple)) and raw else raw
        if actual is not None and (pulse - actual) >= GRIP_STALL_TH:
            break   # contact — stop here

def grip_release(board, lock):
    with lock:
        board.bus_servo_set_position(0.5, [[1, GRIP_OPEN]])

# ── Home ──────────────────────────────────────────────────────────────────────
HOME1_PULSES = {6: 504, 5: 603, 4: 824, 3: 111, 2: 498, 1: 230}

_J1_MAP = [0, 1000, 500, -120,  120,   0]
_J2_MAP = [0, 1000, 500,   30, -210, -90]
_J3_MAP = [0, 1000, 500, -120,  120,   0]
_J4_MAP = [0, 1000, 500,   30, -210, -90]

def _fwd(v, m):
    return ((v - m[2]) / (m[1] - m[0])) * (m[4] - m[3]) + m[5]

def pulses_to_state(p):
    theta = math.radians(_fwd(p[6], _J1_MAP))
    j2    = -math.radians(_fwd(p[5], _J2_MAP)) - math.pi / 2
    j3    =  math.radians(_fwd(p[4], _J3_MAP))
    j4    = -math.radians(_fwd(p[3], _J4_MAP)) - math.pi / 2

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
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

    def publish(self, joints: dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(joints.keys())
        msg.position = list(joints.values())
        self.pub.publish(msg)

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
def fmt_state(theta, pitch, radius, up_down, sens, reachable, note=''):
    tag      = 'OK' if reachable else '!!'
    sens_str = 'MOUSE OFF' if sens == 0 else f'sens×{sens}'
    line = (f"[{tag}]  "
            f"θ={math.degrees(theta):+6.1f}°  "
            f"pitch={math.degrees(pitch):+6.1f}°  "
            f"r={radius*100:5.1f}cm  "
            f"up={up_down*100:+5.1f}cm  "
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
    gripper_frac = 0.0
    sens         = 1
    reachable    = True

    ros_node.publish(solve(theta, pitch, radius, up_down, gripper=gripper_frac).joints)

    print("\nReady.")
    print("  Mouse move    → pan left/right (θ) and up/down")
    print("  Scroll        → radius in / out")
    print("  Left click    → grip     Right click → release")
    print("  Q / E         → pitch up / down")
    print("  H → home1     0 → mouse off     1–9 → sensitivity     Esc → quit\n")
    print(fmt_state(theta, pitch, radius, up_down, sens, reachable))

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    des_theta,  des_pitch,  des_radius,  des_up_down  = theta, pitch, radius, up_down
    smo_theta,  smo_pitch,  smo_radius,  smo_up_down  = theta, pitch, radius, up_down
    sent_theta, sent_pitch, sent_radius, sent_up_down = theta, pitch, radius, up_down

    # Serial port lock — shared between arm command loop and grip thread
    board_lock  = threading.Lock()
    grip_thread = None
    grip_stop   = threading.Event()

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
                            with board_lock:
                                board.bus_servo_set_position(
                                    2.0, [[sid, p] for sid, p in HOME1_PULSES.items()])
                            time.sleep(2.5)
                            t0, p0, r0, u0 = pulses_to_state(HOME1_PULSES)
                            des_theta,  des_pitch,  des_radius,  des_up_down  = t0, p0, r0, u0
                            smo_theta,  smo_pitch,  smo_radius,  smo_up_down  = t0, p0, r0, u0
                            sent_theta, sent_pitch, sent_radius, sent_up_down = t0, p0, r0, u0
                            gripper_frac = 0.0
                            ros_node.publish(solve(t0, p0, r0, u0, gripper=gripper_frac).joints)
                            last_mouse = None
                            last_cmd_t = time.monotonic()
                            reachable  = True
                            note       = '[home1]'
                            _mouse_on()

                        elif key.lower() == 'q':
                            des_pitch += sens * SENS_BASE['pitch']

                        elif key.lower() == 'e':
                            des_pitch -= sens * SENS_BASE['pitch']

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

                        elif btn == 0 and pressed:              # left click → grip
                            if grip_thread is None or not grip_thread.is_alive():
                                grip_stop.clear()
                                grip_thread = threading.Thread(
                                    target=grip_until_stall,
                                    args=(board, board_lock, grip_stop),
                                    daemon=True)
                                grip_thread.start()
                                note = '[gripping]'

                        elif btn == 2 and pressed:              # right click → release
                            grip_stop.set()                     # abort grip if running
                            grip_release(board, board_lock)
                            note = '[released]'

                    more, _, _ = select.select([fd], [], [], 0)
                    if not more:
                        break

            # ── Timed arm command (12 Hz) ─────────────────────────────────────
            now = time.monotonic()
            if now - last_cmd_t >= CMD_INTERVAL:

                smo_theta   = ALPHA * des_theta   + (1 - ALPHA) * smo_theta
                smo_pitch   = ALPHA * des_pitch   + (1 - ALPHA) * smo_pitch
                smo_radius  = ALPHA * des_radius  + (1 - ALPHA) * smo_radius
                smo_up_down = ALPHA * des_up_down + (1 - ALPHA) * smo_up_down

                result    = solve(smo_theta, smo_pitch, smo_radius, smo_up_down,
                                  gripper=gripper_frac)
                reachable = result.reachable

                if reachable:
                    p = result.pulses
                    with board_lock:
                        board.bus_servo_set_position(
                            MOVE_DURATION,
                            [[6, p[6]], [5, p[5]], [4, p[4]], [3, p[3]], [2, p[2]]])
                    ros_node.publish(result.joints)
                    sent_theta, sent_pitch, sent_radius, sent_up_down = (
                        smo_theta, smo_pitch, smo_radius, smo_up_down)
                    if note == '[home1]':
                        note = ''
                else:
                    des_theta,  des_pitch,  des_radius,  des_up_down  = (
                        sent_theta, sent_pitch, sent_radius, sent_up_down)
                    smo_theta,  smo_pitch,  smo_radius,  smo_up_down  = (
                        sent_theta, sent_pitch, sent_radius, sent_up_down)

                last_cmd_t = now

                # Update grip note once thread finishes
                if grip_thread is not None and not grip_thread.is_alive():
                    if note == '[gripping]':
                        note = '[gripped]'

                print(f"\r{fmt_state(smo_theta, smo_pitch, smo_radius, smo_up_down, sens, reachable, note)}   ",
                      end='', flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        grip_stop.set()
        _mouse_off()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n[INFO] Quit.")
        board.port.close()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
