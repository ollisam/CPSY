from gpiozero import Robot
import time
import board
import busio
import adafruit_tcs34725
import math

# --- NEW: distance sensor bits ---
import digitalio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# -----------------------
# Hardware setup (yours)
# -----------------------
robot = Robot(left=('GPIO13','GPIO19'), right=('GPIO12','GPIO18'))

# TCS34725 (line sensor)
i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50   # ms (valid: 2.4, 24, 50, 101, 154, 700)
tcs.gain = 4                # valid gains: 1, 4, 16, 60

# --- NEW: MCP3008 (distance sensor via analog voltage) ---
# Assumes sensor output to MCP3008 channel 0 (P0) and CE0 chip select.
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.CE0)
mcp = MCP.MCP3008(spi, cs)
dist_chan = AnalogIn(mcp, MCP.P0)

# -----------------------
# PID + control params
# ----------------------
Kp = 10
Ki = 1.5
Kd = 90

# The original code used "PWM" in [0..255]. We'll compute in that domain,
# then map to Robot's [-1..1].
BASE_SPEED_PWM = 45

# Clamp speeds to keep some torque but avoid slamming max
MIN_DRIVE_PWM = 30
MIN_PWM = 0
MAX_PWM = 50

# Integral windup guard (same spirit as original)
I_MIN = -4
I_MAX = 4

# If error is tiny, zero the integral to avoid drift
INTEGRAL_DEADZONE = 2.0

# The target "middle" clear channel value (overwritten by calibration)
middle_value = 3300
black_value = 600
white_value = 6000

# PID state
error = 0.0
previous_error = 0.0
sum_error = 0.0

# -----------------------
# Collision avoidance config/state (NEW)
# -----------------------
NEAR_VOLTAGE = 2.0       # volts considered "too close" (~20 cm from your lab)
SAMPLE_INTERVAL = 0.05   # seconds between distance reads (50 ms)
CLEAR_SAMPLES_NEEDED = 3 # require consecutive safe samples before resuming

last_range_sample_t = 0.0
last_voltage = 0.0
blocked = False
clear_count = 0

def too_close(voltage: float) -> bool:
    """Return True if something is within our 'unsafe' range."""
    return voltage >= NEAR_VOLTAGE

# -----------------------
# Utility helpers
# -----------------------
def apply_min_drive(pwm):
    if pwm <= 0:
        return 0
    return max(pwm, MIN_DRIVE_PWM)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def pwm_to_robot_speed(pwm_value):
    pwm_value = clamp(pwm_value, 0, 255)
    pwm_value = (pwm_value / 255.0)
    return pwm_value

def set_motors(left_pwm_signed, right_pwm_signed):
    """
    Accept signed "PWM-like" speeds in range [-255..255],
    map to Robot.value which is tuple in [-1..1].
    """
    left_sign = 1 if left_pwm_signed >= 0 else -1
    right_sign = 1 if right_pwm_signed >= 0 else -1

    left_mag = pwm_to_robot_speed(abs(int(left_pwm_signed)))
    right_mag = pwm_to_robot_speed(abs(int(right_pwm_signed)))

    robot.value = (left_sign * left_mag, right_sign * right_mag)

def stop():
    robot.stop()

def read_clear_channel():
    # Returns (r,g,b,c); we use c
    r, g, b, c = tcs.color_raw
    return r, g, b, c

# -----------------------
# Collision avoidance sampling (NEW)
# -----------------------
def check_obstacle_now():
    """
    Sample the distance sensor if SAMPLE_INTERVAL elapsed.
    Update global blocked/clear_count state.
    """
    global last_range_sample_t, last_voltage, blocked, clear_count

    print(last_range_sample_t)

    now = time.monotonic()
    if (now - last_range_sample_t) < SAMPLE_INTERVAL:
        return  # respect sampling rate

    last_range_sample_t = now
    last_voltage = dist_chan.voltage

    if too_close(last_voltage):
        if not blocked:
            print(f"[COLLISION] Obstacle detected: {last_voltage:0.2f} V -> STOP")
        blocked = True
        clear_count = 0
        stop()
    else:
        if blocked:
            clear_count += 1
            if clear_count >= CLEAR_SAMPLES_NEEDED:
                blocked = False
                clear_count = 0
                print(f"[COLLISION] Clear after {CLEAR_SAMPLES_NEEDED} samples; resuming.")

# -----------------------
# Main control loop
# -----------------------
def loop():
    global error, previous_error, sum_error

    # --- always check obstacle first ---
    check_obstacle_now()

    # If blocked, keep motors off and return early
    if blocked:
        stop()
        return

    # Otherwise, proceed with line following PID
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

    # Compute wheel speeds in "PWM-like" domain
    speed_left = BASE_SPEED_PWM + int(correction)
    speed_right = BASE_SPEED_PWM - int(correction)

    # Clamp
    speed_left = clamp(speed_left, MIN_PWM, MAX_PWM)
    speed_right = clamp(speed_right, MIN_PWM, MAX_PWM)

    speed_left  = apply_min_drive(speed_left)
    speed_right = apply_min_drive(speed_right)

    set_motors(speed_left, speed_right)

def main():
    print("Starting up...")
    # Quick sensor presence sanity check for TCS34725
    try:
        _ = tcs.color_raw
        print("TCS34725 detected.")
    except Exception as e:
        print(f"ERROR: TCS34725 not detected or I2C issue: {e}")
        return

    # Quick sanity read on MCP3008 distance sensor
    try:
        v = dist_chan.voltage
        print(f"MCP3008 distance sensor OK. Initial V={v:0.2f} V")
    except Exception as e:
        print(f"ERROR: MCP3008/analog distance sensor issue: {e}")
        return

    #calibrate_sensor()
    print("Entering control loop. Press Ctrl+C to stop.")

    try:
        while True:
            loop()
            # PID loop cadence; distance sampling is rate-limited internally
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop()

if __name__ == "__main__":
    main()