# toggle_nudge_line_follower.py
# Behavior: every time the sensor crosses the line, alternate nudging with only one wheel.
# Requires: gpiozero, adafruit_tcs34725, board, busio

from gpiozero import Robot
import time
import board
import busio
import adafruit_tcs34725

# ===== Wiring =====
robot = Robot(left=('GPIO13', 'GPIO19'), right=('GPIO12', 'GPIO18'))

# ===== Sensor =====
i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50  # ms
tcs.gain = 4

# ===== Tunables =====
CRAWL_SPEED   = 0.15   # steady crawl speed when not nudging (0..1)
NUDGE_SPEED   = 0.25   # speed for the single-wheel nudge (0..1)
NUDGE_TIME    = 1   # seconds to spin the selected wheel
COOLDOWN      = 0.30   # minimum time between nudge triggers (debounce)
SAMPLE_DT     = 0.02   # sensor poll interval

# Thresholds:
# Option A (quick): fixed thresholds you can tweak
THRESH_WHITE  = 4000.0  # value above this is "white"
THRESH_BLACK  = 3000.0  # value below this is "black"
# Between these, we keep the previous state (hysteresis band)

# If you prefer an interactive 2-point calibration at start, set this True.
INTERACTIVE_CALIBRATION = False

def read_clear():
    # Use the clear (overall brightness) channel
    r, g, b, c = tcs.color_raw
    return float(c)

def calibrate_hysteresis():
    if not INTERACTIVE_CALIBRATION:
        return THRESH_BLACK, THRESH_WHITE

    print("\nCalibration: place sensor over WHITE area, press Enter.")
    input()
    ws = [read_clear() for _ in range(30)]
    white = sum(ws) / len(ws)
    print(f" White level: {white:.1f}")

    print("Place sensor over BLACK line, press Enter.")
    input()
    bs = [read_clear() for _ in range(30)]
    black = sum(bs) / len(bs)
    print(f" Black level: {black:.1f}")

    if white < black:
        white, black = black, white  # ensure white > black if readings inverted

    # Hysteresis: set band between black and white
    lo = black + 0.20 * (white - black)  # lower threshold (closer to black)
    hi = black + 0.50 * (white - black)  # upper threshold (closer to white)
    print(f" Thresholds -> BLACK<{lo:.0f}  WHITE>{hi:.0f}")
    return lo, hi

def main():
    lo, hi = calibrate_hysteresis()
    print("Starting toggle-nudge follower… (Ctrl+C to stop)")

    # Determine initial state with some samples
    samples = [read_clear() for _ in range(10)]
    v = sum(samples)/len(samples)
    on_black = v <= lo  # initial state guess

    # Next nudge direction: True → right wheel, False → left wheel
    nudge_right_next = True

    last_trigger = 0.0

    try:
        while True:
            v = read_clear()

            # Update state with hysteresis
            if on_black and v > hi:
                on_black = False
                crossed = True
            elif (not on_black) and v < lo:
                on_black = True
                crossed = True
            else:
                crossed = False

            now = time.monotonic()

            if crossed and (now - last_trigger) >= COOLDOWN:
                # Perform single-wheel nudge
                if nudge_right_next:
                    # Right wheel nudges forward, left stopped
                    robot.left_motor.stop()
                    robot.right_motor.forward(NUDGE_SPEED)
                else:
                    # Left wheel nudges forward, right stopped
                    robot.right_motor.stop()
                    robot.left_motor.forward(NUDGE_SPEED)

                time.sleep(NUDGE_TIME)
                robot.stop()

                # Alternate for next crossing
                nudge_right_next = not nudge_right_next
                last_trigger = now

                # After nudge, resume crawl
                robot.forward(CRAWL_SPEED)
            else:
                # No crossing: keep crawling forward slowly
                robot.forward(CRAWL_SPEED)

            time.sleep(SAMPLE_DT)

    except KeyboardInterrupt:
        robot.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()