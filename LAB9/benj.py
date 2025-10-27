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
Kp = 0.5
Ki = 0.0
Kd = 0.0

# The original code used "PWM" in [0..255]. We'll compute in that domain,
# then map to Robot's [-1..1].
BASE_SPEED_PWM = 30

# Clamp speeds to keep some torque but avoid slamming max
MIN_PWM = 20
MAX_PWM = 255

# Integral windup guard (same spirit as original)
I_MIN = -4
I_MAX = 4

# If error is tiny, zero the integral to avoid drift
INTEGRAL_DEADZONE = 2.0

# Inner wheel ratio during turns (20% as requested)
INNER_WHEEL_RATIO = 0.2

# The target "middle" clear channel value (overwritten by calibration)
middle_value = 3300
black_value = 600
white_value = 6000

# PID state
error = 0.0
previous_error = 0.0
sum_error = 0.0

# -----------------------
# Utility helpers
# -----------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

MIN_ACTIVE = 0.18   # try 0.18–0.22; must be <= cap (0.20)

def pwm_to_robot_speed(pwm_value):
    pwm_value = clamp(pwm_value, 0, 255)
    mapped = (pwm_value / 255.0) * 0.13
    if pwm_value == 0:
        return 0.0
    return max(mapped, MIN_ACTIVE)

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

    # Robot.value expects (-1..1) per side
    robot.value = (left_sign * left_mag, right_sign * right_mag)

def stop():
    robot.stop()

def read_clear_channel():
    # Returns (r,g,b,c); we use c
    r, g, b, c = tcs.color_raw
    return r, g, b, c

# -----------------------
# New turning logic with inner wheel at 20% power
# -----------------------
def compute_turn_speeds(correction):
    """
    Compute motor speeds for turning with inner wheel at 20% of outer wheel power.
    Returns (left_speed, right_speed) in PWM domain
    """
    if correction > 0:  # Need to turn RIGHT
        # Right turn: left wheel is outer (full power), right wheel is inner (20%)
        outer_speed = BASE_SPEED_PWM + abs(correction)
        inner_speed = outer_speed * INNER_WHEEL_RATIO
       
        left_speed = outer_speed  # Outer wheel (left) gets full power
        right_speed = inner_speed  # Inner wheel (right) gets 20% power
       
    elif correction < 0:  # Need to turn LEFT
        # Left turn: right wheel is outer (full power), left wheel is inner (20%)
        outer_speed = BASE_SPEED_PWM + abs(correction)
        inner_speed = outer_speed * INNER_WHEEL_RATIO
       
        left_speed = inner_speed  # Inner wheel (left) gets 20% power
        right_speed = outer_speed  # Outer wheel (right) gets full power
       
    else:  # Going straight
        left_speed = BASE_SPEED_PWM
        right_speed = BASE_SPEED_PWM
   
    return left_speed, right_speed

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
    previous_error = error

    # Compute wheel speeds using new turning logic
    speed_left, speed_right = compute_turn_speeds(correction)

    # NEW: Apply 20% power to inner wheel during turns
    if correction > 5:  # Turning right significantly
        speed_right = int(speed_left * 0.13)  # Inner wheel (right) at 20%
    elif correction < -5:  # Turning left significantly  
        speed_left = int(speed_right * 0.13)  # Inner wheel (left) at 20%


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

    #calibrate_sensor()
    print("Entering control loop. Press Ctrl+C to stop.")
    print(f"Turning behavior: inner wheel at {INNER_WHEEL_RATIO*100}% of outer wheel power")

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