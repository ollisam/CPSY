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
LOOP_HZ = 25

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

    KP, KI, KD = 0.02, 0.01, 0.001
    pidL = pid.PID(KP, KI, KD, setpoint=0.0)
    pidR = pid.PID(KP, KI, KD, setpoint=0.0)
    for pidc in (pidL, pidR):
        pidc.sample_time = 1.0 / LOOP_HZ
        pidc.output_limits = (0.0, 1.0)

    cruise = False
    setpoint = 0.0
    l_prev, r_prev = safe_steps(left_enc), safe_steps(right_enc)
    t_prev = time.time()

    try:
        while True:
            key = stdscr.getch()
            if key != -1:
                if key in (ord('q'), 27): break
                elif key == ord('p'):
                    cruise = not cruise
                    pidL.reset(); pidR.reset()
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
            l_cps, l_prev, now = cps(l_prev, t_prev, left_enc)
            r_cps, r_prev, now = cps(r_prev, t_prev, right_enc)
            t_prev = now

            if cruise:
                pidL.setpoint = setpoint
                pidR.setpoint = setpoint

                l_out = pidL(l_cps)
                r_out = pidR(r_cps)

                set_left(l_out)
                set_right(r_out)
            else:
                stop_all()

            # display info
            stdscr.addstr(0, 2, "q=quit p=PID w/s=setpoint 1/2=Kp 3/4=Ki 5/6=Kd")
            stdscr.addstr(1, 2, f"PID: {'ON ' if cruise else 'OFF'}  setpoint={setpoint:6.1f}")
            stdscr.addstr(2, 2, f"Kp={KP:.4f}  Ki={KI:.4f}  Kd={KD:.4f}")
            stdscr.addstr(3, 2, f"L cps={l_cps:7.1f} out={left_pwm.value:5.2f}")
            stdscr.addstr(4, 2, f"R cps={r_cps:7.1f} out={right_pwm.value:5.2f}")
            stdscr.refresh()
            time.sleep(1.0 / LOOP_HZ)
    finally:
        stop_all()
        curses.nocbreak(); stdscr.keypad(False); curses.echo(); curses.endwin()

if __name__ == "__main__":
    print("=== PID calibration tool for DRV8833 ===")
    print("Use: w/s=setpoint | 1/2=Kp± | 3/4=Ki± | 5/6=Kd± | p=start | q=quit")
    curses.wrapper(main)