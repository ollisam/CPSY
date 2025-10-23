# pd_edge_follow.py
from gpiozero import Robot
import time
import board, busio
import adafruit_tcs34725

robot = Robot(left=('GPIO13','GPIO19'), right=('GPIO12','GPIO18'))

# ---- Sensor ----
i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50
tcs.gain = 4

# ---- Tunables ----
BASE_SPEED = 0.20     # forward crawl
Kp         = 1.0      # proportional on reflectance error
Kd         = 0.25     # derivative dampening
MAX_TURN   = 0.6      # cap on steering effort
SAMPLE_DT  = 0.02     # seconds

# Set this to 'left' or 'right' depending on which edge you want to follow.
# If it steers the wrong way, flip this value (or swap sign of Kp).
FOLLOW_EDGE = 'left'  # 'left' means we keep the left side of the line under sensor

def read_clear():
    r,g,b,c = tcs.color_raw
    return float(c)

def calibrate():
    print("Calibration: put sensor over WHITE, press Enter.")
    input()
    w = sum(read_clear() for _ in range(30))/30.0
    print(f"  White: {w:.1f}")
    print("Now put sensor over BLACK line, press Enter.")
    input()
    b = sum(read_clear() for _ in range(30))/30.0
    print(f"  Black: {b:.1f}")
    if w < b:  # ensure white > black
        w, b = b, w
    target = b + 0.50*(w - b)   # edge = mid reflectance
    scale  = max(1.0, (w - b))  # normalize error
    print(f"Target(mid): {target:.1f}")
    return b, w, target, scale

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

def main():
    *_ignore, target, scale = calibrate()
    print("PD edge-following… Ctrl+C to stop")
    prev = read_clear()
    prev_t = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            val = read_clear()
            dt = max(1e-3, now - prev_t)

            # error: positive when we're lighter than target (too much white)
            e = (target - val) / scale

            # If following the LEFT edge, positive error should turn LEFT (slow left wheel).
            # If following the RIGHT edge, invert the sign so positive error turns RIGHT.
            sign = +1.0 if FOLLOW_EDGE == 'left' else -1.0
            de = (prev - val) / (scale * dt)  # derivative of error (approx)
            steer = sign*(Kp*e + Kd*de)
            steer = clamp(steer, -MAX_TURN, MAX_TURN)

            # differential drive: left = base - steer, right = base + steer
            left = clamp(BASE_SPEED - steer, 0.0, 1.0)
            right = clamp(BASE_SPEED + steer, 0.0, 1.0)

            # If you want to enforce “only one wheel at a time”, uncomment:
            # if left > right: right = 0.0
            # else: left = 0.0

            robot.value = (left, right)

            prev, prev_t = val, now
            time.sleep(SAMPLE_DT)

    except KeyboardInterrupt:
        robot.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()