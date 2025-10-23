# line_follow_zigzag_gpiozero.py
from gpiozero import Robot
import time
import board
import busio
import adafruit_tcs34725
from math import copysign

# === CONFIG ===
# Motor wiring: (left motor forward/backward, right motor forward/backward)
robot = Robot(left=('GPIO13', 'GPIO19'), right=('GPIO12', 'GPIO18'))

# I2C color sensor
i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)

# --- ADAPTIVE THRESHOLD + PROBE TUNING ---
THRESHOLD_FACTOR = 0.60   # fraction of current "white" level to consider black
EMA_ALPHA        = 0.05   # how fast we update the running white estimate when not on black
PROBE_TIME       = 0.08   # s, short nudge when line is lost to sense darker direction
PROBE_SPEED      = 0.16   # wheel speed during probe
PROBE_IMPROVE    = 0.03   # required fractional drop (>=3%) in brightness to accept probe direction

# Sensor tuning
tcs.integration_time = 50   # ms
tcs.gain = 4

# Speed settings
FORWARD_SPEED = 0.15
TURN_SPEED    = 0.20

# Timing / thresholds
SAMPLE_DT     = 0.02
LOST_AFTER    = 0.15

SWEEP_START   = 0.20
SWEEP_STEP    = 0.10
SWEEP_MAX     = 0.50

# Search behavior tuning
BACKUP_SPEED  = 0.15   # speed while backing up during search
BACKUP_TIME   = 0.4   # seconds to back up after each arc sweep when not found
ARC_BIAS      = 0.5    # 0..1, how much slower the inner wheel is during arc (higher => tighter turn)

# ---- Helpers ----

def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def calibrate_white(samples: int = 30, dt: float = 0.01) -> float:
    """Sample the sensor to estimate the current background (white) level.
    Ask the user to place the sensor over the track background before starting.
    """
    vals = []
    for _ in range(samples):
        vals.append(read_brightness())
        time.sleep(dt)
    return sum(vals) / len(vals)


def probe_direction() -> str | None:
    """When the line is lost, do two tiny yaw probes and pick the darker direction.
    Returns 'left', 'right', or None if inconclusive.
    """
    baseline = read_brightness()

    # Left probe: slow/stop left wheel, move right wheel
    robot.value = (0.0, PROBE_SPEED)
    time.sleep(PROBE_TIME)
    left_b = read_brightness()
    robot.stop()
    time.sleep(0.02)

    if left_b < baseline * (1.0 - PROBE_IMPROVE):
        return 'left'

    # Right probe
    robot.value = (PROBE_SPEED, 0.0)
    time.sleep(PROBE_TIME)
    right_b = read_brightness()
    robot.stop()
    time.sleep(0.02)

    if right_b < baseline * (1.0 - PROBE_IMPROVE):
        return 'right'

    return None


def drive_arc(direction: str, outer: float, bias: float):
    """Drive an arc by slowing the inner wheel according to bias (0..1).
    direction: 'left' or 'right'
    outer: outer wheel speed
    bias: 0..1, inner wheel = outer*(1-bias)
    """
    inner = clamp(outer * (1.0 - bias))
    if direction == 'left':
        robot.value = (inner, outer)
    else:
        robot.value = (outer, inner)

# ---- Sensor helpers ----
def read_brightness():
    _, _, _, clear = tcs.color_raw
    return float(clear)

def is_black(value, threshold):
    return value <= threshold

# ---- Main loop ----
def main():
    print("Calibrating background… Place sensor over track background (not the black line).")
    white_est = calibrate_white()
    threshold = white_est * THRESHOLD_FACTOR

    print(f"Starting line following with adaptive threshold ~ {threshold:.0f} (Ctrl+C to stop)")

    last_seen = time.monotonic()
    sweep_len = SWEEP_START
    sweep_dir = 'left'  # alternate when inconclusive

    try:
        while True:
            brightness = read_brightness()
            now = time.monotonic()
            threshold = white_est * THRESHOLD_FACTOR

            if is_black(brightness, threshold):
                # On the line → go straight and shorten next sweep
                robot.forward(FORWARD_SPEED)
                last_seen = now
                sweep_len = SWEEP_START
                time.sleep(SAMPLE_DT)
                continue

            # Off the line: treat this as background and slowly update white estimate
            white_est = (1.0 - EMA_ALPHA) * white_est + EMA_ALPHA * brightness

            if (now - last_seen) < LOST_AFTER:
                # Recently saw the line → keep rolling forward gently
                robot.forward(FORWARD_SPEED * 0.8)
                time.sleep(SAMPLE_DT)
                continue

            # LOST: stop and search using probe-guided arcs
            robot.stop()

            # Quick direction probe
            probe = probe_direction()
            if probe is not None:
                sweep_dir = probe

            # Arc search with expanding duration
            found = False
            drive_arc(sweep_dir, TURN_SPEED, ARC_BIAS)
            end_time = time.monotonic() + sweep_len
            while time.monotonic() < end_time:
                if is_black(read_brightness(), threshold):
                    found = True
                    break
                time.sleep(SAMPLE_DT)
            robot.stop()

            if found:
                # Reacquired → nudge forward, reset sweep
                robot.forward(FORWARD_SPEED)
                last_seen = time.monotonic()
                sweep_len = SWEEP_START
                time.sleep(0.10)
            else:
                # Not found → small backup to avoid drifting, then flip side and grow sweep
                robot.backward(BACKUP_SPEED)
                time.sleep(BACKUP_TIME)
                robot.stop()

                sweep_dir = 'right' if sweep_dir == 'left' else 'left'
                sweep_len = min(SWEEP_MAX, sweep_len + SWEEP_STEP)
                time.sleep(0.03)

    except KeyboardInterrupt:
        robot.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()