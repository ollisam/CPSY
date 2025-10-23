# line_follow_stop_and_scan_gpiozero.py
from gpiozero import Robot
import time
import board
import busio
import adafruit_tcs34725
from collections import deque

# === CONFIG ===
# Motor wiring: (left motor forward/backward, right motor forward/backward)
robot = Robot(left=('GPIO13', 'GPIO19'), right=('GPIO12', 'GPIO18'))

# I2C color sensor
i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)

# Sensor tuning
tcs.integration_time = 50   # ms
tcs.gain = 4                # try 4–16 depending on ambient light

# Speed settings
FORWARD_SPEED = 0.35        # ~35–40% like you wanted
TURN_SPEED    = 0.28        # spin speed for scanning; keep moderate

# Timing / thresholds
SAMPLE_DT     = 0.02        # control loop ~50 Hz
LOST_DEBOUNCE = 0.15        # must be "off" for 150 ms to be considered lost
SCAN_SIDE_TIME = 1.0        # max seconds to scan on one side before swapping

# Thresholds (will be set by calibrate(); fallback constants are here)
USE_AUTOCAL   = True
BLACK_LEVEL   = 2500        # fallback raw "clear" value over black
WHITE_LEVEL   = 12000       # fallback raw "clear" value over white
ON_THRESH     = None        # computed: value <= ON_THRESH => on black line
OFF_THRESH    = None        # computed: value >= OFF_THRESH => off the line (hysteresis band)

# Smoothing
AVG_WINDOW    = 5           # moving average samples

# ---- Sensor helpers ----
def read_brightness():
    # TCS34725 raw "clear" channel
    _, _, _, clear = tcs.color_raw
    return float(clear)

def calibrate():
    """
    Quick two-point calibration:
      1) Place sensor over BLACK line when prompted.
      2) Then place over WHITE background when prompted.
    Computes ON/OFF thresholds with a small hysteresis band.
    """
    global BLACK_LEVEL, WHITE_LEVEL, ON_THRESH, OFF_THRESH

    print("Calibration: place sensor over BLACK line... (sampling)")
    time.sleep(1.0)
    blacks = [read_brightness() for _ in range(60)]
    BLACK_LEVEL = sum(blacks)/len(blacks)
    print(f"  black avg = {BLACK_LEVEL:.0f}")

    print("Calibration: now place over WHITE background... (sampling)")
    time.sleep(1.0)
    whites = [read_brightness() for _ in range(60)]
    WHITE_LEVEL = sum(whites)/len(whites)
    print(f"  white avg = {WHITE_LEVEL:.0f}")

    # Safety order
    if WHITE_LEVEL < BLACK_LEVEL:
        WHITE_LEVEL, BLACK_LEVEL = BLACK_LEVEL, WHITE_LEVEL

    # Thresholds with hysteresis (10% of span)
    span = max(100.0, WHITE_LEVEL - BLACK_LEVEL)
    mid = (WHITE_LEVEL + BLACK_LEVEL) * 0.5
    band = 0.10 * span
    ON_THRESH  = mid - band * 0.5   # <= ON means we're on the dark line
    OFF_THRESH = mid + band * 0.5   # >= OFF means we're clearly off it

    print(f"Thresholds set: ON<= {ON_THRESH:.0f}   OFF>= {OFF_THRESH:.0f}")

def set_thresholds_from_fallback():
    global ON_THRESH, OFF_THRESH
    span = max(100.0, WHITE_LEVEL - BLACK_LEVEL)
    mid = (WHITE_LEVEL + BLACK_LEVEL) * 0.5
    band = 0.10 * span
    ON_THRESH  = mid - band * 0.5
    OFF_THRESH = mid + band * 0.5
    print(f"(No calibration) Using thresholds: ON<= {ON_THRESH:.0f}  OFF>= {OFF_THRESH:.0f}")

def on_line(x):   # x is brightness (lower = darker)
    return x <= ON_THRESH

def off_line(x):
    return x >= OFF_THRESH

# ---- State machine ----
class State:
    FOLLOW = 0
    SCAN_RIGHT = 1
    SCAN_LEFT  = 2

def main():
    # Calibrate or use defaults
    if USE_AUTOCAL:
        calibrate()
    else:
        set_thresholds_from_fallback()

    print("Starting line following... (Ctrl+C to stop)")
    state = State.FOLLOW
    last_seen_time = time.monotonic()
    last_seen_side = +1    # +1 means last known drift to the RIGHT; -1 to the LEFT
    scan_start = None

    # small moving average for brightness
    avgq = deque(maxlen=AVG_WINDOW)

    try:
        robot.forward(FORWARD_SPEED)

        while True:
            b = read_brightness()
            avgq.append(b)
            b_sm = sum(avgq)/len(avgq)

            now = time.monotonic()

            if state == State.FOLLOW:
                if on_line(b_sm):
                    last_seen_time = now
                    # Update last_seen_side using tiny heuristic: if brightness rising vs falling
                    # (optional; comment out if not useful with your track)
                    # last_seen_side = +1 if derivative > 0 else -1
                elif off_line(b_sm):
                    # Debounce "lost"
                    if (now - last_seen_time) >= LOST_DEBOUNCE:
                        # Stop and start scanning toward last_seen_side first
                        robot.stop()
                        if last_seen_side >= 0:
                            state = State.SCAN_RIGHT
                        else:
                            state = State.SCAN_LEFT
                        scan_start = time.monotonic()
                        continue
                # keep rolling
                robot.forward(FORWARD_SPEED)

            elif state == State.SCAN_RIGHT:
                # in-place clockwise spin
                robot.right(TURN_SPEED)

                if on_line(b_sm):
                    robot.stop()
                    time.sleep(0.05)  # short settle
                    robot.forward(FORWARD_SPEED)
                    last_seen_time = time.monotonic()
                    last_seen_side = +1
                    state = State.FOLLOW
                    continue

                # Timeout → try the other side
                if time.monotonic() - scan_start > SCAN_SIDE_TIME:
                    robot.stop()
                    state = State.SCAN_LEFT
                    scan_start = time.monotonic()
                    continue

            elif state == State.SCAN_LEFT:
                # in-place counter-clockwise spin
                robot.left(TURN_SPEED)

                if on_line(b_sm):
                    robot.stop()
                    time.sleep(0.05)
                    robot.forward(FORWARD_SPEED)
                    last_seen_time = time.monotonic()
                    last_seen_side = -1
                    state = State.FOLLOW
                    continue

                if time.monotonic() - scan_start > SCAN_SIDE_TIME:
                    robot.stop()
                    state = State.SCAN_RIGHT
                    scan_start = time.monotonic()
                    continue

            time.sleep(SAMPLE_DT)

    except KeyboardInterrupt:
        robot.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()
