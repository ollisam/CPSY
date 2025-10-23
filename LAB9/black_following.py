# line_follow_zigzag.py
import time
import board
import busio
import adafruit_tcs34725

# ---- MOTOR SETUP (you already have these instances) ----
# Replace these with your actual objects if they're constructed elsewhere.
class Motor:
    def __init__(self, fwd_pin, rev_pin):
        # Placeholder for whatever your real Motor does
        self._speed = 0.0
    def forward(self, speed: float):
        self._speed = max(0.0, min(1.0, speed))
        # TODO: implement GPIO/PWM for forward
    def backward(self, speed: float):
        self._speed = max(0.0, min(1.0, speed))
        # TODO: implement GPIO/PWM for backward
    def stop(self):
        self._speed = 0.0
        # TODO: implement stop (both pins off)

# If you already construct these elsewhere, comment these two lines and import instead.
left  = Motor('GPIO13', 'GPIO19')  # (forward, backward)
right = Motor('GPIO12', 'GPIO18')

# ---- I2C + COLOR SENSOR ----
i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)   # default addr 0x29
# A little smoothing helps with noise
tcs.integration_time = 50   # ms
tcs.gain = 4                # 1, 4, 16, 60 are typical

# ---- TUNABLES ----
FORWARD_SPEED = 0.35        # linear drive speed on the line
TURN_SPEED    = 0.30        # pivot speed while searching
SAMPLE_DT     = 0.02        # seconds between brightness samples
LOST_AFTER    = 0.15        # seconds off-line before we begin zig-zag
SWEEP_START   = 0.10        # initial pivot burst (s)
SWEEP_STEP    = 0.05        # how much longer each alternate sweep gets
SWEEP_MAX     = 0.50        # cap the pivot burst length
MARGIN        = 0.05        # safety margin below threshold (fraction of range)

# ---- MOTOR HELPERS (edit here if your Motor API is different) ----
def stop():
    left.stop(); right.stop()

def forward(speed=FORWARD_SPEED):
    # Simple differential forward
    left.forward(speed)
    right.forward(speed)

def pivot_left(speed=TURN_SPEED):
    # Left wheel backward, right forward = left pivot
    left.backward(speed)
    right.forward(speed)

def pivot_right(speed=TURN_SPEED):
    # Right wheel backward, left forward = right pivot
    left.forward(speed)
    right.backward(speed)

# ---- SENSOR / CALIBRATION ----
def read_brightness():
    """
    Return normalized brightness using the clear channel (0.0 ~ 1.0).
    """
    r, g, b, c = tcs.color_raw  # c is 'clear' (overall intensity)
    # Simple normalization to 0..1 based on a plausible raw range.
    # We'll re-map using calibrated white/black anyway.
    return float(c)

def calibrate_threshold():
    print("\nCalibration: place the sensor over WHITE background, then press Enter.")
    input()
    white_samples = []
    for _ in range(30):
        white_samples.append(read_brightness())
        time.sleep(0.02)
    white = sum(white_samples) / len(white_samples)
    print(f"  White level: {white:.1f}")

    print("Now place the sensor over the BLACK line, then press Enter.")
    input()
    black_samples = []
    for _ in range(30):
        black_samples.append(read_brightness())
        time.sleep(0.02)
    black = sum(black_samples) / len(black_samples)
    print(f"  Black level: {black:.1f}")

    if white <= black:
        # Fallback if readings are inverted or environment odd
        white, black = max(white, black), min(white, black)

    # Threshold a bit above black toward white; margin shrinks false positives.
    thr = black + (white - black) * (0.35)  # bias toward detecting "dark"
    # Apply extra safety margin downward (treat 'darker than thr*(1 - MARGIN)' as black)
    eff_thr = thr * (1.0 - MARGIN)

    print(f"Computed threshold: {thr:.1f} | effective: {eff_thr:.1f}")
    return black, white, eff_thr

def is_black(brightness, threshold):
    return brightness <= threshold

# ---- MAIN CONTROL LOOP ----
def main():
    print("Line Follow (zig-zag) starting… Ctrl+C to stop.")
    black, white, THRESH = calibrate_threshold()
    print("Following the line…")

    last_seen = time.monotonic()
    sweep_len = SWEEP_START
    sweep_left = True

    try:
        while True:
            b = read_brightness()
            now = time.monotonic()

            if is_black(b, THRESH):
                # On the line: go forward, reset search state
                forward(FORWARD_SPEED)
                last_seen = now
                sweep_len = SWEEP_START
                # small sample delay
                time.sleep(SAMPLE_DT)
                continue

            # Off the line but recently seen: keep easing forward briefly
            if (now - last_seen) < LOST_AFTER:
                forward(FORWARD_SPEED * 0.8)
                time.sleep(SAMPLE_DT)
                continue

            # Fully lost: begin/continue zig-zag search
            stop()
            found = False

            # Try one pivot burst in current direction while sampling
            burst_end = time.monotonic() + sweep_len
            if sweep_left:
                pivot_left(TURN_SPEED)
            else:
                pivot_right(TURN_SPEED)

            while time.monotonic() < burst_end:
                b = read_brightness()
                if is_black(b, THRESH):
                    found = True
                    break
                time.sleep(SAMPLE_DT)

            stop()

            if found:
                # Snap forward to re-center; then resume main loop
                forward(FORWARD_SPEED)
                last_seen = time.monotonic()
                sweep_len = SWEEP_START  # reset sweep size after a hit
                time.sleep(0.08)         # tiny settle forward
            else:
                # Alternate direction and grow sweep up to max
                sweep_left = not sweep_left
                sweep_len = min(SWEEP_MAX, sweep_len + SWEEP_STEP)
                # brief pause to avoid momentum issues
                time.sleep(0.03)

    except KeyboardInterrupt:
        pass
    finally:
        stop()
        print("Stopped.")

if __name__ == "__main__":
    main()