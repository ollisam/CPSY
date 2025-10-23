# Simple single-sensor line follower (black line on light surface)
# Requires: gpiozero, adafruit-circuitpython-tcs34725, Adafruit Blinka
# If you already instantiate `robot` and `i2c` elsewhere, remove those lines here.

import time
from gpiozero import Robot
import board
import busio
import adafruit_tcs34725

# --- Motor wiring (GPIO pin numbers are BCM) ---
robot = Robot(left=('GPIO13', 'GPIO19'), right=('GPIO12', 'GPIO18'))

# --- I2C color sensor ---
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)
sensor.integration_time = 24  # ms (lower = faster updates)
sensor.gain = 4               # 1, 4, 16, 60 (adjust if needed)

# --- Tunables ---
BASE_SPEED = 0.2      # 0..1 forward speed
K_STEER = 0.8         # steering gain; raise if it drifts, lower if too twitchy
MIN_SPEED = 0.15      # keep motors from stalling on turns
SAMPLE_DT = 0.02      # loop delay, ~50 Hz control

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def read_intensity():
    # Use the "clear" channel as a simple reflectance proxy
    r, g, b, c = sensor.color_raw
    return c

def calibrate_white_level(duration=1.0):
    # Assume we start on the light surface; measure average intensity
    t_end = time.time() + duration
    samples, total = 0, 0.0
    while time.time() < t_end:
        total += read_intensity()
        samples += 1
        time.sleep(0.01)
    return (total / max(1, samples))

def main():
    print("Calibrating on light background...")
    # white_level = calibrate_white_level(1.0)
    # Consider anything 30% darker than 'white' as the line
    threshold = 2674.3
    # print(f"White level ~ {white_level:.1f}, threshold ~ {threshold:.1f}")
    print("Following line. Press Ctrl+C to stop.")

    # simple low-pass on intensity to reduce jitter
    filt = read_intensity()

    try:
        while True:
            raw = read_intensity()
            # exponential moving average
            filt = 0.6 * filt + 0.4 * raw

            # Error is how far below/above threshold we are (negative = too bright, positive = too dark)
            # We want to steer toward the darker line.
            error = (threshold - filt) / max(1.0, threshold)

            # Convert error into differential steering
            turn = K_STEER * error
            # left wheel slower when on the line (darker), right wheel faster -> small left correction
            left = clamp(BASE_SPEED - turn, MIN_SPEED, 1.0)
            right = clamp(BASE_SPEED + turn, MIN_SPEED, 1.0)

            robot.value = (left, right)
            time.sleep(SAMPLE_DT)

    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        print("Stopped.")

if __name__ == "__main__":
    main()