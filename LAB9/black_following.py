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

# Search behavior tuning
BACKUP_SPEED  = 0.15   # speed while backing up during search
BACKUP_TIME   = 0.25   # seconds to back up after each arc sweep when not found
ARC_BIAS      = 0.5    # 0..1, how much slower the inner wheel is during arc (higher => tighter turn)

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

            # Lost line → begin zig-zag search (arc + backup pattern)
            robot.stop()
            found = False

            # compute arc speeds: inner wheel slower by ARC_BIAS
            inner = max(0.0, TURN_SPEED * (1.0 - ARC_BIAS))
            outer = TURN_SPEED

            # Choose direction based on sweep_left
            if sweep_left:
                # gpiozero: use robot.value=(left,right) for independent wheel speeds; forward() doesn't accept a tuple.
                robot.value = (inner, outer)
            else:
                # forward-right arc: left wheel = outer, right wheel = inner
                robot.value = (outer, inner)

            end_time = time.monotonic() + sweep_len
            while time.monotonic() < end_time:
                if is_black(read_brightness(), threshold):
                    found = True
                    break
                time.sleep(SAMPLE_DT)

            robot.stop()

            if found:
                # Reacquired: drive ahead briefly to re-center and reset sweep
                robot.forward(FORWARD_SPEED)
                last_seen = time.monotonic()
                sweep_len = SWEEP_START
                time.sleep(0.08)
            else:
                # Not found: back up a little to avoid drifting off course, then try the other side
                robot.backward(BACKUP_SPEED)
                time.sleep(BACKUP_TIME)
                robot.stop()

                sweep_left = not sweep_left
                sweep_len = min(SWEEP_MAX, sweep_len + SWEEP_STEP)
                time.sleep(0.03)

    except KeyboardInterrupt:
        robot.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()