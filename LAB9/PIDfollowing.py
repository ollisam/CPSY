from gpiozero import Robot
import time
import board
import busio
import adafruit_tcs34725
import math

# -----------------------
# Hardware setup (yours)
# -----------------------
robot = Robot(left=('GPIO13','GPIO19'), right=('GPIO12','GPIO18'))

i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50   # ms (valid options in this lib include 2.4, 24, 50, 101, 154, 700)
tcs.gain = 4                # valid gains: 1, 4, 16, 60

# -----------------------
# PID + control params
# -----------------------
Kp = 24
Ki = 1
Kd = 1

# --- Global speed limits (for gpiozero outputs) ---
SPEED_CAP = 0.35        # 0..1 hard ceiling for motor magnitude (lower = slower)
MIN_ACTIVE = 0.08       # deadband to prevent buzzing at very low speeds

# Optional: limit how much PID can add/subtract per loop (in PWM-like units)
CORRECTION_LIMIT_PWM = 80

# The original code used "PWM" in [0..255]. We'll compute in that domain,
# then map to Robot's [-1..1].
BASE_SPEED_PWM = 20

# Clamp speeds to keep some torque but avoid slamming max
MIN_PWM = 0
MAX_PWM = 255

# Adaptive slowdown and slight asymmetry to prevent right-turn overshoot
TURN_SLOWDOWN = 0.8   # fraction of base speed to potentially shed (0..1)
RIGHT_TURN_SCALE = 0.7  # scale down right-turn correction (>0) to avoid overshoot
# Use the known white/black spread to normalize how aggressive the slowdown is
# (safe to compute later as well if black/white updated)

# Integral windup guard (same spirit as original)
I_MIN = -4
I_MAX = 4

# If error is tiny, zero the integral to avoid drift
INTEGRAL_DEADZONE = 2.0

# The target "middle" clear channel value (overwritten by calibration)
middle_value = 2500
black_value = 500
white_value = 5000

# PID state
error = 0.0
previous_error = 0.0
sum_error = 0.0

# -----------------------
# Utility helpers
# -----------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def pwm_to_robot_speed(pwm_value):
    """
    Convert 0..255 PWM-ish magnitude to 0..1 speed for gpiozero.
    """
    pwm_value = clamp(pwm_value, 0, 255)
    # Map 0..255 to 0..SPEED_CAP so we have a global top speed limit
    return (pwm_value / 255.0) * SPEED_CAP

def set_motors(left_pwm_signed, right_pwm_signed):
    """
    Accept signed "PWM-like" speeds in range [-255..255],
    map to Robot.value which is tuple in [-1..1].
    """
    # Direction + magnitude split
    left_sign = 1 if left_pwm_signed >= 0 else -1
    right_sign = 1 if right_pwm_signed >= 0 else -1

    left_mag = pwm_to_robot_speed(abs(int(left_pwm_signed)))
    right_mag = pwm_to_robot_speed(abs(int(right_pwm_signed)))

    # Apply a small deadband to avoid motor buzzing at very low duty
    if 0 < left_mag < MIN_ACTIVE:
        left_mag = 0.0
    if 0 < right_mag < MIN_ACTIVE:
        right_mag = 0.0

    # Robot.value expects (-1..1) per side, already capped by SPEED_CAP
    robot.value = (left_sign * left_mag, right_sign * right_mag)

def stop():
    robot.stop()

def read_clear_channel():
    # Returns (r,g,b,c); we use c
    r, g, b, c = tcs.color_raw
    return r, g, b, c

# -----------------------
# Calibration
# -----------------------
# def calibrate_sensor():
#     """
#     Spin a bit to sample 'black' and 'white' under the sensor,
#     then compute the midpoint.
#     Movement here mirrors your Arduino: small opposite wheel spins.
#     """
#     global middle_value

#     print("Calibrating: measuring black...")
#     # turn in place (left backward, right forward)
#     _, _, _, c_black = read_clear_channel()
#     print(f"Black value: {c_black}")

#     time.sleep(5)

#     print("Calibrating: measuring white...")
#     # turn the other way (left forward, right backward)
#     _, _, _, c_white = read_clear_channel()
#     print(f"White value: {c_white}")

#     time.sleep(5)

#     middle_value = (int(c_black) + int(c_white)) // 2
#     print(f"Middle value = {middle_value}")

#     stop()

# -----------------------
# Main control loop
# -----------------------
def loop():
    global error, previous_error, sum_error

    # Read sensor
    _, _, _, c = read_clear_channel()

    # PID
    error = float(middle_value) - float(c)
    sum_error += error
    sum_error = clamp(sum_error, I_MIN, I_MAX)

    if abs(error) < INTEGRAL_DEADZONE:
        sum_error = 0.0

    differential = error - previous_error
    correction = Kp * error + Ki * sum_error + Kd * differential
    correction = clamp(correction, -CORRECTION_LIMIT_PWM, CORRECTION_LIMIT_PWM)
    previous_error = error

    # Slightly tame right turns to avoid overshooting the black line
    if correction > 0:
        correction *= RIGHT_TURN_SCALE

    # Compute a dynamic base that slows down as the error grows
    # Estimate an error scale using half the measured white/black spread
    err_den = max(1.0, abs((white_value - black_value) * 0.5))
    err_factor = min(1.0, abs(error) / err_den)
    dynamic_base = int(BASE_SPEED_PWM * (1.0 - TURN_SLOWDOWN * err_factor))

    # Compute wheel speeds in "PWM-like" domain with adaptive base
    speed_left = dynamic_base + int(correction)
    speed_right = dynamic_base - int(correction)

    # Clamp
    speed_left = clamp(speed_left, MIN_PWM, MAX_PWM)
    speed_right = clamp(speed_right, MIN_PWM, MAX_PWM)

    # Apply
    set_motors(speed_left, speed_right)

def main():
    print("Starting up...")
    # Quick sensor presence sanity check
    try:
        _ = tcs.color_raw
        print("TCS34725 detected.")
    except Exception as e:
        print(f"ERROR: TCS34725 not detected or I2C issue: {e}")
        return

    # calibrate_sensor()
    print("Entering control loop. Press Ctrl+C to stop.")

    try:
        while True:
            loop()
            # Match typical Arduino loop cadence (integration time is 50ms; keep a modest pace)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop()

if __name__ == "__main__":
    main()