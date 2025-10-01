import os
import time
import curses
import pid  # simple-pid wrapper provided in the lab (PID class available as pid.PID)

from gpiozero import DigitalOutputDevice, Motor, Robot, RotaryEncoder

"""
PID-based cruise control for the RC robot.

Controls:
  q : quit
  e : emergency stop (also exits PID mode)
  p : toggle PID cruise mode on/off
  w / s : increase / decrease PID target speed (counts/s)
  a / d : small left / right trim while in PID mode (differential setpoint)
  Arrow keys / WASD : manual tank-style movement when PID mode is OFF (as before)

Notes:
- Encoders: set the GPIO pins below to match your wiring.
- The PID runs on *counts per second* (cps) from each wheel encoder.
- Output limits are [0, 1] which maps to the motor speed value for each side.
"""

# === CONFIG: GPIO PINS (CHANGE THESE TO MATCH YOUR HARDWARE) ===
LEFT_MOTOR_PINS = ("GPIO19", "GPIO13")  # (forward, backward)
RIGHT_MOTOR_PINS = ("GPIO18", "GPIO12")

LEFT_ENCODER_PINS = ("GPIO21", "GPIO20")  # (A, B)  <-- TODO: set correctly
RIGHT_ENCODER_PINS = ("GPIO6", "GPIO16")   # (A, B)  <-- TODO: set correctly

# === CONFIG: PID GAINS (TUNE THESE) ===
KP = 0.02
KI = 0.10
KD = 0.001

# Target speed step when pressing w/s (counts per second)
SETPOINT_STEP = 10.0
# Small differential trim for a/d while in PID mode
TRIM_STEP = 3.0


def _read_steps(encoder):
    """Safely read the encoder step count (gpiozero.RotaryEncoder exposes .steps)."""
    return getattr(encoder, "steps", 0)


def _cps(prev_steps, prev_time, encoder):
    """Return counts per second and new (steps, time)."""
    now = time.time()
    steps = _read_steps(encoder)
    dt = max(1e-3, now - prev_time)
    cps = (steps - prev_steps) / dt
    return cps, steps, now


try:
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)
    stdscr.nodelay(1)

    stdscr.addstr(0, 2, "Hit 'q' to quit | 'p' PID on/off | 'e' STOP")

    # Motors/Robot
    robot = Robot(left=Motor(*LEFT_MOTOR_PINS), right=Motor(*RIGHT_MOTOR_PINS))

    # Encoders (A,B)
    left_enc = RotaryEncoder(a=LEFT_ENCODER_PINS[0], b=LEFT_ENCODER_PINS[1])
    right_enc = RotaryEncoder(a=RIGHT_ENCODER_PINS[0], b=RIGHT_ENCODER_PINS[1])

    # PID controllers (one per wheel)
    target_left = 0.0
    target_right = 0.0

    pid_left = pid.PID(KP, KI, KD, setpoint=target_left)
    pid_right = pid.PID(KP, KI, KD, setpoint=target_right)
    # Run the PIDs every loop iteration (we handle dt ourselves)
    pid_left.sample_time = None
    pid_right.sample_time = None
    # Motor command range [0, 1]
    pid_left.output_limits = (0.0, 1.0)
    pid_right.output_limits = (0.0, 1.0)

    # PID mode toggle
    pid_enabled = False

    # Encoder history for cps
    ls_prev_steps = _read_steps(left_enc)
    rs_prev_steps = _read_steps(right_enc)
    t_prev_l = t_prev_r = time.time()

    direction = None
    last_info = ""  # line cache for UI

    while direction != ord('q'):
        stdscr.refresh()
        direction = stdscr.getch()

        # --- EMERGENCY STOP ---
        if direction == ord('e'):
            pid_enabled = False
            target_left = target_right = 0.0
            pid_left.setpoint = target_left
            pid_right.setpoint = target_right
            robot.stop()
            last_info = "STOP"

        # --- TOGGLE PID MODE ---
        if direction == ord('p'):
            pid_enabled = not pid_enabled
            mode = "PID ON" if pid_enabled else "PID OFF"
            last_info = mode

        if pid_enabled:
            # Adjust setpoints while PID is active
            if direction == ord('w'):
                target_left += SETPOINT_STEP
                target_right += SETPOINT_STEP
            if direction == ord('s'):
                target_left = max(0.0, target_left - SETPOINT_STEP)
                target_right = max(0.0, target_right - SETPOINT_STEP)
            if direction == ord('a'):
                target_left = max(0.0, target_left - TRIM_STEP)
                target_right += TRIM_STEP
            if direction == ord('d'):
                target_left += TRIM_STEP
                target_right = max(0.0, target_right - TRIM_STEP)

            # Update PID setpoints
            pid_left.setpoint = target_left
            pid_right.setpoint = target_right

            # Measure speeds (counts per second)
            lcps, ls_prev_steps, t_prev_l = _cps(ls_prev_steps, t_prev_l, left_enc)
            rcps, rs_prev_steps, t_prev_r = _cps(rs_prev_steps, t_prev_r, right_enc)

            # Compute control actions
            u_left = pid_left(lcps)
            u_right = pid_right(rcps)

            # Drive forward with computed speeds
            robot.value = (u_left, u_right)

            last_info = f"PID  set(L,R)={target_left:.1f},{target_right:.1f}  cps(L,R)={lcps:.1f},{rcps:.1f}  out(L,R)={u_left:.2f},{u_right:.2f}"

        else:
            # Manual drive (original behavior)
            if direction == ord('a'):
                robot.left()
                last_info = "Manual: Left     "
            if direction == ord('d'):
                robot.right()
                last_info = "Manual: Right    "
            if direction == ord('w'):
                robot.forward()
                last_info = "Manual: Forward  "
            if direction == ord('s'):
                robot.backward()
                last_info = "Manual: Backward "
            if direction == ord('e'):
                robot.stop()
                last_info = "Manual: Stop     "

        # UI
        stdscr.addstr(1, 2, last_info.ljust(78))
        stdscr.addstr(2, 2, f"Mode: {'PID' if pid_enabled else 'Manual'} | w/s = speed +/- | a/d = trim | p = toggle PID ")

        time.sleep(0.04)

finally:
    # Important to set everything back by end of the script
    curses.nocbreak()
    try:
        stdscr.keypad(0)
    except Exception:
        pass
    curses.endwin()
