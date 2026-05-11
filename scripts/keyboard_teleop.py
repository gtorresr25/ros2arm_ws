#!/usr/bin/env python3
"""
ArmPi Ultra — keyboard end-effector teleop

Controls
--------
  W / S         Z up / down
  Q / E         Base rotate left / right  (rotates the [x,y] vector, keeps reach)
  ↑ / ↓         Reach forward / backward  (X axis)
  ← / →         Wrist pitch tilt -/+
  Z / X         Gripper open / close
  C             Toggle torque engage / release
  H             Return to home position
  1 – 9         Step size multiplier (1 = 1 mm / 2°, 9 = 9 mm / 18°)
  P             Print current position
  Esc / Ctrl-C  Quit

Usage
-----
  source /opt/ros/jazzy/setup.bash
  python3 keyboard_teleop.py
"""

import sys
import tty
import select
import termios
import math
import time

SDK_PATH = ("/home/andres/ros2arm_ws/ArmPi_Ultra_Resources"
            "/Source Code/ROS2/src/driver/ros_robot_controller"
            "/ros_robot_controller")
sys.path.insert(0, SDK_PATH)

try:
    from ros_robot_controller_sdk import Board
except ImportError as e:
    print(f"[ERROR] SDK not found: {e}")
    sys.exit(1)

try:
    from kinematics.ik import solve, fk
except ImportError as e:
    print(f"[ERROR] kinematics package not found: {e}")
    print("        Run:  source /opt/ros/jazzy/setup.bash && source install/setup.bash")
    sys.exit(1)

# ── Hardware ──────────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 1_000_000
SERVO_IDS     = [6, 5, 4, 3, 2]     # IK pulse index 0→4 maps to these servo IDs
GRIPPER_ID    = 1

# ── Motion ────────────────────────────────────────────────────────────────────
MOVE_DURATION       = 0.15           # seconds per command
TRANSLATION_STEP    = 0.001          # 1 mm per unit
ROTATION_STEP       = 2.0            # 2 degrees per unit
DEFAULT_STEP        = 5              # default multiplier (keys 1-9 override)

# ── Gripper pulses ────────────────────────────────────────────────────────────
GRIPPER_OPEN        = 200
GRIPPER_CLOSE       = 800

# ── Safe workspace (meters / degrees) ────────────────────────────────────────
X_MIN, X_MAX        = 0.05,  0.28
Y_MIN, Y_MAX        = -0.20, 0.20
Z_MIN, Z_MAX        = 0.01,  0.35
PITCH_MIN, PITCH_MAX = -90.0, 90.0

# ── Starting end-effector pose ────────────────────────────────────────────────
START_XYZ   = [0.18, 0.0, 0.20]     # meters
START_PITCH = 0.0                    # degrees

# ── Pulse change limits ───────────────────────────────────────────────────────
# Known home2 pulses — arm pointing straight up.  Used as the IK warm-start
# seed at startup so the first solve stays near the current physical pose.
HOME2_PULSES  = {6: 504, 5: 510, 4: 496, 3: 502, 2: 499, 1: 230}

# Reject any IK result where a single servo would jump more than this many
# pulses. Guards against ikpy flipping to a different elbow branch.
MAX_PULSE_JUMP = 80

# Duration scaling: seconds per pulse of maximum servo travel.
# 80 pulses → 0.15 s (snappy), 200 pulses → 0.37 s (still safe).
SEC_PER_PULSE  = 0.15 / 80

# ── Key escape sequences ──────────────────────────────────────────────────────
KEY_UP    = '\x1b[A'
KEY_DOWN  = '\x1b[B'
KEY_RIGHT = '\x1b[C'
KEY_LEFT  = '\x1b[D'


# ─────────────────────────────────────────────────────────────────────────────

def _read_key() -> str:
    """Read one keypress (including arrow sequences) from stdin in raw mode."""
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        # Check if more bytes follow within 50 ms (arrow key sequence)
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return '\x1b[' + ch3
            return '\x1b' + ch2
        # Plain Escape — no follow-on bytes
        return '\x1b'
    return ch


def _print_controls() -> None:
    print(
        "\n"
        "  W / S       Z up / down\n"
        "  Q / E       base rotate left / right\n"
        "  ↑ / ↓       reach forward / backward\n"
        "  ← / →       wrist pitch  -/+\n"
        "  Z / X       gripper open / close\n"
        "  C           toggle torque\n"
        "  H           home position\n"
        "  1–9         step size  (current: {step})\n"
        "  P           print position\n"
        "  Esc/Ctrl-C  quit\n"
    )


# ─────────────────────────────────────────────────────────────────────────────

class ArmTeleop:

    def __init__(self):
        print(f"[INFO] Connecting to arm on {SERIAL_PORT} …")
        self.board = Board(device=SERIAL_PORT, baudrate=BAUD_RATE)
        self.board.enable_reception(True)
        time.sleep(0.3)
        print("[OK]  Connected.")

        self.pos            = list(START_XYZ)
        self.pitch          = START_PITCH
        self.step           = DEFAULT_STEP
        self.torque_on      = False             # unknown at start — engage below
        self.current_pulses = dict(HOME2_PULSES)  # warm-start seed; tracks last sent

        self._engage()
        print(f"[INFO] Moving to start position {self.pos} …")
        if not self._send_ik():
            print("[WARN] IK failed for start position — check arm is powered.")

    # ── Torque helpers ────────────────────────────────────────────────────────

    def _engage(self) -> None:
        """Load (engage) torque on all servos including gripper."""
        for sid in [1, 2, 3, 4, 5, 6]:
            self.board.bus_servo_enable_torque(sid, False)  # False = LOAD
            time.sleep(0.02)
        self.torque_on = True

    def _release(self) -> None:
        """Unload (release) torque on all servos."""
        for sid in [1, 2, 3, 4, 5, 6]:
            self.board.bus_servo_enable_torque(sid, True)   # True = UNLOAD
            time.sleep(0.02)
        self.torque_on = False

    # ── IK → servo command ────────────────────────────────────────────────────

    def _send_ik(self) -> bool:
        """Run IK for current pos and send servo command. Returns success."""
        x, y, z = self.pos
        result = solve(x, y, z, current_pulses=self.current_pulses)
        if result is None:
            return False

        # Reject if ikpy jumped to a different arm configuration.
        max_delta = max(
            abs(result[sid] - self.current_pulses.get(sid, result[sid]))
            for sid in SERVO_IDS
        )
        if max_delta > MAX_PULSE_JUMP:
            return False

        # Scale duration so larger moves stay smooth.
        duration = max(MOVE_DURATION, max_delta * SEC_PER_PULSE)

        self.current_pulses = result
        cmd = [[sid, result[sid]] for sid in SERVO_IDS]
        self.board.bus_servo_set_position(duration, cmd)
        return True

    # ── Boundary clamp ────────────────────────────────────────────────────────

    def _clamp(self) -> None:
        self.pos[0] = max(X_MIN,     min(X_MAX,     self.pos[0]))
        self.pos[1] = max(Y_MIN,     min(Y_MAX,     self.pos[1]))
        self.pos[2] = max(Z_MIN,     min(Z_MAX,     self.pos[2]))
        self.pitch  = max(PITCH_MIN, min(PITCH_MAX, self.pitch))

    # ── Key handler ───────────────────────────────────────────────────────────

    def handle_key(self, key: str) -> bool:
        """
        Process one keypress. Returns False if the user wants to quit,
        True otherwise.
        """
        t  = self.step * TRANSLATION_STEP
        r  = self.step * ROTATION_STEP

        # Save state so we can revert on IK failure
        prev_pos   = list(self.pos)
        prev_pitch = self.pitch
        needs_move = False

        # ── Translation / rotation ──────────────────────────────────────────
        if key == 'w':
            self.pos[2] += t;  needs_move = True
        elif key == 's':
            self.pos[2] -= t;  needs_move = True

        elif key == KEY_UP:
            self.pos[0] += t;  needs_move = True
        elif key == KEY_DOWN:
            self.pos[0] -= t;  needs_move = True

        elif key == KEY_RIGHT:
            self.pitch  += r;  needs_move = True
        elif key == KEY_LEFT:
            self.pitch  -= r;  needs_move = True

        elif key in ('q', 'e'):
            # Rotate the [x, y] vector by ±r degrees around Z, keeping reach.
            reach = math.sqrt(self.pos[0] ** 2 + self.pos[1] ** 2)
            if reach < 1e-6:
                reach = 0.001          # degenerate guard
            theta = math.atan2(self.pos[1], self.pos[0])
            delta = math.radians(r) * (1 if key == 'q' else -1)
            theta += delta
            self.pos[0] = reach * math.cos(theta)
            self.pos[1] = reach * math.sin(theta)
            needs_move = True

        # ── Gripper ─────────────────────────────────────────────────────────
        elif key == 'z':
            self.board.bus_servo_set_position(
                MOVE_DURATION, [[GRIPPER_ID, GRIPPER_OPEN]])
            print(f"\r  gripper: OPEN   ", end='', flush=True)

        elif key == 'x':
            self.board.bus_servo_set_position(
                MOVE_DURATION, [[GRIPPER_ID, GRIPPER_CLOSE]])
            print(f"\r  gripper: CLOSE  ", end='', flush=True)

        # ── Torque toggle ────────────────────────────────────────────────────
        elif key == 'c':
            if self.torque_on:
                self._release()
                print("\r  torque: RELEASED (arm limp)   ", end='', flush=True)
            else:
                self._engage()
                print("\r  torque: ENGAGED               ", end='', flush=True)

        # ── Home ─────────────────────────────────────────────────────────────
        elif key == 'h':
            self.pos   = list(START_XYZ)
            self.pitch = START_PITCH
            self._clamp()
            self._send_ik()
            print(f"\r  homed → {self._fmt_pos()}   ", end='', flush=True)

        # ── Step size ────────────────────────────────────────────────────────
        elif key in '123456789':
            self.step = int(key)
            print(f"\r  step: {self.step} "
                  f"({self.step * TRANSLATION_STEP * 1000:.0f} mm / "
                  f"{self.step * ROTATION_STEP:.0f}°)   ",
                  end='', flush=True)

        # ── Print position ───────────────────────────────────────────────────
        elif key == 'p':
            print(f"\r  {self._fmt_pos()}   ", end='', flush=True)

        # ── Quit ─────────────────────────────────────────────────────────────
        elif key in ('\x1b', '\x03'):   # Esc or Ctrl-C
            return False

        # ── Apply move ───────────────────────────────────────────────────────
        if needs_move:
            self._clamp()
            if not self._send_ik():
                # IK has no solution — revert and warn
                self.pos   = prev_pos
                self.pitch = prev_pitch
                print("\r  [!] no IK solution, position reverted   ",
                      end='', flush=True)
            else:
                print(f"\r  {self._fmt_pos()}   ", end='', flush=True)

        return True

    def _fmt_pos(self) -> str:
        x, y, z = [v * 1000 for v in self.pos]
        return (f"x={x:+6.1f}mm  y={y:+6.1f}mm  z={z:+6.1f}mm  "
                f"pitch={self.pitch:+6.1f}°  step={self.step}")

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self) -> None:
        print("\n[INFO] Keyboard teleop running. Esc or Ctrl-C to quit.")
        print(
            "\n"
            "  W/S  Z↕    Q/E  rotate base    ↑/↓  reach    ←/→  wrist pitch\n"
            "  Z/X  gripper    C  torque    H  home    1-9  step    P  print pos\n"
        )

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                key = _read_key()
                if not self.handle_key(key):
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("\n[INFO] Quitting — re-engaging torque …")
            self._engage()
            self.board.port.close()
            print("[OK]  Done.")


# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    teleop = ArmTeleop()
    teleop.run()


if __name__ == '__main__':
    main()