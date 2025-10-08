# movedaca.py
import time
import curses
import pid
from gpiozero import Motor, Robot, RotaryEncoder

# === CONFIG: BCM pin numbers (ints) ===
LEFT_MOTOR_PINS  = (22, 18)   # left: forward, backward
RIGHT_MOTOR_PINS = (13, 17)   # right: forward, backward

LEFT_ENCODER_PINS  = (20, 21)  # A, B
RIGHT_ENCODER_PINS = (6, 16)   # A, B

# If forward motion shows NEGATIVE cps, set the sign to -1 (your case).
ENCODER_SIGN_LEFT  = -1
ENCODER_SIGN_RIGHT = -1

# PID gains (tune on hardware)
KP, KI, KD = 0.014, 0.4, 0.01   # start with KI=0 to avoid windup while testing
SETPOINT_STEP = 10.0             # cps per press of w/s
TRIM_STEP = 3.0                  # cps per press of a/d
LOOP_HZ = 25                     # control loop frequency (25 Hz ~ 40 ms)

# Optional: minimum command to overcome motor dead-zone (0 disables)
UMIN = 0.0  # e.g., try 0.18 if wheels won't move at low speed

def apply_deadzone(u, umin=UMIN):
    if umin <= 0.0:
        return u
    return 0.0 if u <= 0.0 else max(umin, min(1.0, u))

def safe_steps(enc):
    try:
        return enc.steps
    except Exception:
        return 0

def cps(prev_steps, prev_time, enc):
    now = time.time()
    steps = safe_steps(enc)
    dt = max(1e-3, now - prev_time)
    return (steps - prev_steps) / dt, steps, now

def main(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.clear()
    stdscr.addstr(0, 2, "q: quit  p: PID on/off  e: STOP  w/s: speed ±   a/d: trim ±")

    # Motors (enable PWM!)
    left_motor  = Motor(LEFT_MOTOR_PINS[0], LEFT_MOTOR_PINS[1], pwm=True)
    right_motor = Motor(RIGHT_MOTOR_PINS[0], RIGHT_MOTOR_PINS[1], pwm=True)
    robot = Robot(left=left_motor, right=right_motor)

    # Encoders
    left_enc  = RotaryEncoder(*LEFT_ENCODER_PINS, max_steps=0)
    right_enc = RotaryEncoder(*RIGHT_ENCODER_PINS, max_steps=0)

    # Two independent PIDs (one per wheel)
    left_pid = pid.PID(KP, KI, KD, setpoint=0.0)
    right_pid = pid.PID(KP, KI, KD, setpoint=0.0)
    for pidc in (left_pid, right_pid):
        pidc.sample_time = 1.0 / LOOP_HZ
        pidc.output_limits = (0.0, 1.0)   # forward-only speed 0..1

    # State
    cruise_enabled = False
    base_setpoint = 0.0   # cps
    trim = 0.0            # cps (left-right differential)

    # Encoder history for cps
    l_prev_steps, r_prev_steps = safe_steps(left_enc), safe_steps(right_enc)
    prev_t = time.time()

    try:
        while True:
            # --- input ---
            key = -1
            try:
                key = stdscr.getch()
            except Exception:
                pass

            if key != -1:
                if key in (ord('q'), 27):  # q or ESC
                    break
                elif key == ord('e'):
                    cruise_enabled = False
                    base_setpoint = 0.0
                    trim = 0.0
                    robot.stop()
                elif key == ord('p'):
                    cruise_enabled = not cruise_enabled
                    if cruise_enabled:
                        left_pid.reset()
                        right_pid.reset()
                elif key == ord('w'):
                    base_setpoint += SETPOINT_STEP
                    left_pid.reset(); right_pid.reset()
                elif key == ord('s'):
                    base_setpoint = max(0.0, base_setpoint - SETPOINT_STEP)
                    left_pid.reset(); right_pid.reset()
                elif key == ord('a'):
                    trim -= TRIM_STEP
                elif key == ord('d'):
                    trim += TRIM_STEP

            # --- measure cps from encoders ---
            l_cps, l_prev_steps, now = cps(l_prev_steps, prev_t, left_enc)
            r_cps, r_prev_steps, now = cps(r_prev_steps, prev_t, right_enc)
            prev_t = now

            # Make forward positive
            l_cps *= ENCODER_SIGN_LEFT
            r_cps *= ENCODER_SIGN_RIGHT

            # --- control ---
            if cruise_enabled:
                left_pid.setpoint  = max(0.0, base_setpoint + trim)
                right_pid.setpoint = max(0.0, base_setpoint - trim)

                # Compute new motor commands (0..1)
                l_cmd = left_pid(l_cps)
                r_cmd = right_pid(r_cps)

                # Optional dead-zone compensation
                l_cmd = apply_deadzone(l_cmd)
                r_cmd = apply_deadzone(r_cmd)

                # Apply (forward-only)
                left_motor.forward(l_cmd)
                right_motor.forward(r_cmd)
            else:
                robot.stop()

            # --- UI ---
            stdscr.addstr(2, 2, f"PID: {'ON ' if cruise_enabled else 'OFF'}   base_setpoint={base_setpoint:6.1f} cps   trim={trim:+5.1f} cps   ")
            stdscr.addstr(3, 2, f"MEAS: left={l_cps:7.1f} cps   right={r_cps:7.1f} cps   ")
            stdscr.addstr(4, 2, f"OUT : left={left_motor.value:5.2f}   right={right_motor.value:5.2f}   ")
            stdscr.refresh()

            # --- loop timing ---
            time.sleep(1.0 / LOOP_HZ)
    finally:
        robot.stop()
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

if __name__ == "__main__":
    curses.wrapper(main)
