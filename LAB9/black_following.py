# line_follow_zigzag_gpiozero.py
from gpiozero import Robot
import time
import board
import busio
import adafruit_tcs34725

# === CONFIG ===
# Motor wiring: (left motor forward/backward, right motor forward/backward)
robot = Robot(left=('GPIO13', 'GPIO19'), right=('GPIO12', 'GPIO18'))

# I2C color sensor
i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)

# Sensor tuning
tcs.integration_time = 50   # ms
tcs.gain = 4

# Speed settings
FORWARD_SPEED = 0.15
TURN_SPEED    = 0.15

# Timing / thresholds
SAMPLE_DT     = 0.02
LOST_AFTER    = 0.15
SWEEP_START   = 0.10
SWEEP_STEP    = 0.05
SWEEP_MAX     = 0.50

# ---- Sensor helpers ----
def read_brightness():
    _, _, _, clear = tcs.color_raw
    return float(clear)

def is_black(value, threshold):
    return value <= threshold

# ---- Main loop ----
def main():
    threshold = 3000
    print("Starting line following... (Ctrl+C to stop)")

    last_seen = time.monotonic()
    sweep_left = True
    sweep_len = SWEEP_START

    try:
        while True:
            brightness = read_brightness()
            now = time.monotonic()

            if is_black(brightness, threshold):
                robot.forward(FORWARD_SPEED)
                last_seen = now
                sweep_len = SWEEP_START
                time.sleep(SAMPLE_DT)
                continue

            if (now - last_seen) < LOST_AFTER:
                robot.forward(FORWARD_SPEED * 0.8)
                time.sleep(SAMPLE_DT)
                continue

            # Lost line → begin zig-zag search
            robot.stop()
            found = False

            end_time = now + sweep_len
            if sweep_left:
                robot.left(TURN_SPEED)
            else:
                robot.right(TURN_SPEED)

            while time.monotonic() < end_time:
                if is_black(read_brightness(), threshold):
                    found = True
                    break
                time.sleep(SAMPLE_DT)

            robot.stop()

            if found:
                robot.forward(FORWARD_SPEED)
                last_seen = time.monotonic()
                sweep_len = SWEEP_START
                time.sleep(0.08)
            else:
                sweep_left = not sweep_left
                sweep_len = min(SWEEP_MAX, sweep_len + SWEEP_STEP)
                time.sleep(0.03)

    except KeyboardInterrupt:
        robot.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()