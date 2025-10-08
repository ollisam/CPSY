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
MIN_PWM   = 0.08   # minimum duty to overcome static friction
FF_KS     = 0.0   # static feedforward (approx. duty to just start turning)
FF_KV     = 0.0008 # velocity feedforward per cps (tune to your enc. scale)
CPS_ALPHA = 0.3    # EMA filter for cps; higher = less smoothing

def apply_deadzone(u):
    # Negative or tiny -> 0; allow real zero speed
    if u <= 0.0: 
        return 0.0
    if u < MIN_PWM:
        return 0.0
    return min(1.0, u)

# ===== Motor helpers =====
left_pwm  = PWMOutputDevice(LEFT_PWM, frequency=PWM_HZ, initial_value=0.0)
left_dir  = DigitalOutputDevice(LEFT_DIR, initial_value=False)
right_pwm = PWMOutputDevice(RIGHT_PWM, frequency=PWM_HZ, initial_value=0.0)
right_dir = DigitalOutputDevice(RIGHT_DIR, initial_value=False)

def set_left(v):
    left_dir.off()
    left_pwm.value = max(0.0, min(1.0, v))

def set_right(v):
    right_dir.off()
    right_pwm.value = max(0.0, min(1.0, v))

def stop_all():
    set_left(0.0)
    set_right(0.0)

def safe_steps(enc):
    try: return enc.steps
    except Exception: return 0

def cps(prev_steps, prev_time, enc):
    now = time.time()
    steps = safe_steps(enc)
    dt = max(1e-3, now - prev_time)
    return (steps - prev_steps) / dt, steps, now

# ===== Calibration main loop =====
def main(stdscr):
    curses.cbreak(); stdscr.nodelay(True)
    stdscr.keypad(True); stdscr.clear()

    left_enc  = RotaryEncoder(*LEFT_ENCODER_PINS, max_steps=0)
    right_enc = RotaryEncoder(*RIGHT_ENCODER_PINS, max_steps=0)

    KP, KI, KD = 0.05, 0.00, 0.002
    pidL = pid.PID(KP, KI, KD, setpoint=0.0)
    pidR = pid.PID(KP, KI, KD, setpoint=0.0)
    for pidc in (pidL, pidR):
        pidc.sample_time = 1.0 / LOOP_HZ
        pidc.output_limits = (-1, 1)  # PID trims around feedforward; narrower to avoid hunting

    cruise = False
    setpoint = 0.0
    l_prev, r_prev = safe_steps(left_enc), safe_steps(right_enc)
    t_prev = time.time()

    l_cps_f, r_cps_f = 0.0, 0.0

    try:
        while True:
            key = stdscr.getch()
            if key != -1:
                if key in (ord('q'), 27): break
                elif key == ord('p'):
                    cruise = not cruise
                    pidL.reset(); pidR.reset()
                    l_cps_f, r_cps_f = 0.0, 0.0
                    if not cruise: stop_all()
                elif key == ord('w'):
                    setpoint += 10
                elif key == ord('s'):
                    setpoint = max(0.0, setpoint - 10)
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

            # measure cps

            now = time.time()
            l_cps, l_prev = (safe_steps(left_enc) - l_prev) / (now - t_prev), safe_steps(left_enc)
            r_cps, r_prev = (safe_steps(right_enc) - r_prev) / (now - t_prev), safe_steps(right_enc)
            t_prev = now

            # l_cps, l_prev, now = cps(l_prev, t_prev, left_enc)
            # r_cps, r_prev, now = cps(r_prev, t_prev, right_enc)
            # t_prev = now
            # Exponential moving average to reduce encoder noise / quantization
            l_cps_f = (1.0 - CPS_ALPHA) * l_cps_f + CPS_ALPHA * l_cps
            r_cps_f = (1.0 - CPS_ALPHA) * r_cps_f + CPS_ALPHA * r_cps

            if cruise:
                pidL.setpoint = setpoint
                pidR.setpoint = setpoint

                # Purely velocity-based feedforward
                base_ff = 0.0 if setpoint <= 0 else min(1.0, FF_KV * setpoint)

                # PID on filtered cps; outputs are small trims around feedforward
                l_trim = pidL(l_cps_f)
                r_trim = pidR(r_cps_f)

                l_cmd = apply_deadzone(base_ff + l_trim) if setpoint > 0.0 else 0.0
                r_cmd = apply_deadzone(base_ff + r_trim) if setpoint > 0.0 else 0.0

                set_left(l_cmd)
                set_right(r_cmd)
            else:
                stop_all()

            # display info
            stdscr.addstr(0, 2, "q=quit p=PID w/s=setpoint 1/2=Kp 3/4=Ki 5/6=Kd")
            stdscr.addstr(1, 2, f"PID: {'ON ' if cruise else 'OFF'}  setpoint={setpoint:6.1f}")
            stdscr.addstr(2, 2, f"Kp={KP:.4f}  Ki={KI:.4f}  Kd={KD:.4f}")
            stdscr.addstr(3, 2, f"L cps={l_cps_f:7.1f} out={left_pwm.value:5.2f}")
            stdscr.addstr(4, 2, f"R cps={r_cps_f:7.1f} out={right_pwm.value:5.2f}")
            stdscr.refresh()
            time.sleep(1.0 / LOOP_HZ)
    finally:
        stop_all()
        curses.nocbreak(); stdscr.keypad(False); curses.echo(); curses.endwin()

if __name__ == "__main__":
    print("=== PID calibration tool for DRV8833 ===")
    print("Use: w/s=setpoint | 1/2=Kp± | 3/4=Ki± | 5/6=Kd± | p=start | q=quit")
    curses.wrapper(main)