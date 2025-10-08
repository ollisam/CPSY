# pid_calibrate.py  —  PID tuning tool for DRV8833 motors
import time
import curses
import pid
from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder

# ===== DRV8833 PIN SETUP =====
LEFT_PWM = 22    # AIN1 (PWM)
LEFT_DIR = 18    # AIN2 (digital)
RIGHT_PWM = 13   # BIN1 (PWM)
RIGHT_DIR = 17   # BIN2 (digital)
LEFT_ENCODER_PINS  = (20, 21)
RIGHT_ENCODER_PINS = (6, 16)

PWM_HZ = 1000
LOOP_HZ = 50

# ===== Control helpers =====
MIN_PWM   = 0.08    # minimum duty considered “real”; below this we send 0
FF_KS     = 0.0     # static feedforward (disabled; use pure velocity FF)
FF_KV     = 0.003  # velocity feedforward per cps (tune to your encoder/drive)
CPS_ALPHA = 0.3     # EMA filter for cps; higher = less smoothing

TRIM_STEP   = 0.25   # steering step per keypress
STEER_LIMIT = 1.0    # clamp |steer| ≤ 1

def apply_deadzone(u: float) -> float:
    """Clamp negatives to 0 and zero-out tiny positive commands instead of forcing a minimum.
    This allows truly low speeds and avoids jumping to a plateau."""
    if u <= 0.0:
        return 0.0
    if u < MIN_PWM:
        return 0.0
    return min(1.0, u)

# ===== Motor helpers =====
left_pwm  = PWMOutputDevice(LEFT_PWM,  frequency=PWM_HZ, initial_value=0.0)
left_dir  = DigitalOutputDevice(LEFT_DIR,  initial_value=False)
right_pwm = PWMOutputDevice(RIGHT_PWM, frequency=PWM_HZ, initial_value=0.0)
right_dir = DigitalOutputDevice(RIGHT_DIR, initial_value=False)

def set_left(v: float):
    left_dir.off()  # forward
    left_pwm.value = max(0.0, min(1.0, v))

def set_right(v: float):
    right_dir.off()  # forward
    right_pwm.value = max(0.0, min(1.0, v))

def stop_all():
    set_left(0.0)
    set_right(0.0)

def safe_steps(enc: RotaryEncoder) -> int:
    try:
        return enc.steps
    except Exception:
        return 0

# ===== Calibration main loop =====
def main(stdscr):
    curses.cbreak(); stdscr.nodelay(True)
    stdscr.keypad(True); stdscr.clear()

    left_enc  = RotaryEncoder(*LEFT_ENCODER_PINS, max_steps=0)
    right_enc = RotaryEncoder(*RIGHT_ENCODER_PINS, max_steps=0)

    # Small trims only; start conservative to avoid blasting to max speed
    KP, KI, KD = 0.5, 0.0000, 0.0000
    pidL = pid.PID(KP, KI, KD, setpoint=0.0)
    pidR = pid.PID(KP, KI, KD, setpoint=0.0)
    for pidc in (pidL, pidR):
        pidc.sample_time = 1.0 / LOOP_HZ
        pidc.output_limits = (-0.2, 0.2)  # keep trims small so FF dominates

    cruise = False
    setpoint = 0.0
    steer = 0.0  # -1.0 = hard left, +1.0 = hard right

    # step history for cps
    l_prev = safe_steps(left_enc)
    r_prev = safe_steps(right_enc)
    t_prev = time.time()

    # filtered cps
    l_cps_f, r_cps_f = 0.0, 0.0

    try:
        while True:
            key = stdscr.getch()
            if key != -1:
                if key in (ord('q'), 27):
                    break
                elif key == ord('p'):
                    cruise = not cruise
                    pidL.reset(); pidR.reset()
                    l_cps_f, r_cps_f = 0.0, 0.0
                    if not cruise:
                        stop_all()
                elif key == ord('w'):
                    setpoint += 10
                elif key == ord('s'):
                    setpoint = max(0.0, setpoint - 10)
                elif key == ord('a'):
                    steer = max(-STEER_LIMIT, steer - TRIM_STEP)
                elif key == ord('d'):
                    steer = min(STEER_LIMIT, steer + TRIM_STEP)
                elif key == ord('c'):
                    steer = 0.0  # center steering
                elif key == ord('1'):
                    KP *= 1.2
                elif key == ord('2'):
                    KP /= 1.2
                elif key == ord('3'):
                    KI *= 1.2
                elif key == ord('4'):
                    KI /= 1.2
                elif key == ord('5'):
                    KD *= 1.2
                elif key == ord('6'):
                    KD /= 1.2

                pidL.tunings = (KP, KI, KD)
                pidR.tunings = (KP, KI, KD)

            # ===== measure cps with a single shared dt =====
            now = time.time()
            dt = max(1e-3, now - t_prev)
            l_steps_now = safe_steps(left_enc)
            r_steps_now = safe_steps(right_enc)
            l_cps = (l_steps_now - l_prev) / dt
            r_cps = (r_steps_now - r_prev) / dt
            l_prev, r_prev, t_prev = l_steps_now, r_steps_now, now

            # Exponential moving average to reduce quantization noise
            l_cps_f = (1.0 - CPS_ALPHA) * l_cps_f + CPS_ALPHA * l_cps
            r_cps_f = (1.0 - CPS_ALPHA) * r_cps_f + CPS_ALPHA * r_cps

            if cruise:
                # Differential steering: steer∈[-1,1] biases left/right speeds
                # Hard left  (steer=-1) -> left ≈ 0×setpoint, right ≈ 2×setpoint (clamped by FF/output limits)
                # Hard right (steer=+1) -> left ≈ 2×setpoint, right ≈ 0×setpoint
                base_sp = max(0.0, setpoint)
                l_sp = base_sp * max(0.0, 1.0 + steer)
                r_sp = base_sp * max(0.0, 1.0 - steer)

                pidL.setpoint = l_sp
                pidR.setpoint = r_sp

                # Purely velocity-based feedforward so command scales with each wheel's setpoint
                base_ff_l = 0.0 if l_sp <= 0 else min(1.0, FF_KV * l_sp)
                base_ff_r = 0.0 if r_sp <= 0 else min(1.0, FF_KV * r_sp)

                # PID trims around feedforward using filtered cps
                l_trim = pidL(l_cps_f)
                r_trim = pidR(r_cps_f)

                # raw commands before deadzone
                raw_l = base_ff_l + l_trim
                raw_r = base_ff_r + r_trim

                # Deadzone zeros tiny outputs; allows true low speeds
                l_cmd = raw_l if l_sp > 0.0 else 0.0
                r_cmd = raw_r if r_sp > 0.0 else 0.0

                set_left(l_cmd)
                set_right(r_cmd)
            else:
                stop_all()

            # ===== display info =====
            stdscr.addstr(0, 2, "q=quit p=PID w/s=setpoint a/d=steer± c=center 1/2=Kp 3/4=Ki 5/6=Kd")
            stdscr.addstr(1, 2, f"PID: {'ON ' if cruise else 'OFF'}  setpoint={setpoint:6.1f}  steer={steer:+.2f}")
            stdscr.addstr(2, 2, f"Kp={KP:.4f}  Ki={KI:.4f}  Kd={KD:.4f}")
            stdscr.addstr(3, 2, f"L cps={l_cps:7.1f} (f={l_cps_f:7.1f}) out={left_pwm.value:5.2f}")
            stdscr.addstr(4, 2, f"R cps={r_cps_f:7.1f} out={right_pwm.value:5.2f}")
            # Feedforward and trims to diagnose saturation/overpower
            try:
                stdscr.addstr(5, 2, f"LFF={base_ff_l:4.2f} RFF={base_ff_r:4.2f} "
                                     f"Ltrim={l_trim:5.2f} Rtrim={r_trim:5.2f} "
                                     f"rawL={raw_l:5.2f} rawR={raw_r:5.2f}")
            except NameError:
                # before first cruise loop, these may not exist yet
                stdscr.addstr(5, 2, "LFF=-- RFF=-- Ltrim=-- Rtrim=-- rawL=-- rawR=--")

            stdscr.refresh()
            time.sleep(1.0 / LOOP_HZ)
    finally:
        stop_all()
        curses.nocbreak(); stdscr.keypad(False); curses.echo(); curses.endwin()

if __name__ == "__main__":
    print("=== PID calibration tool for DRV8833 ===")
    print("Use: w/s=setpoint | 1/2=Kp± | 3/4=Ki± | 5/6=Kd± | p=start | q=quit")
    curses.wrapper(main)
