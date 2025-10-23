import time
from collections import deque
from gpiozero import Robot
# assume you initialized TCS34725 earlier as `tcs`, with calibration done

robot = Robot(left=('GPIO13','GPIO19'), right=('GPIO12','GPIO18'))

BASE_SPEED = 0.4          # 0.30–0.40 of full speed as you suggested
TURN_SPEED = 0.35         # slow steady spin
SAMPLE_DT = 0.02          # 50 Hz control
ON_THRESH  = 0.35         # normalized reflectance: <= on line
OFF_THRESH = 0.45         # >= off line (hysteresis)
LOST_DEBOUNCE = 0.15      # seconds outside band before we say "lost"

# simple moving average to reduce flicker
avg = deque(maxlen=5)
last_seen_side = 0         # -1=left, +1=right, 0=unknown

def read_reflectance_norm():
    # use tcs.color_raw -> (r,g,b,c), normalize with your calibrated black/white
    r,g,b,c = tcs.color_raw
    brightness = c
    norm = (brightness - BLACK_CAL) / max(1, (WHITE_CAL - BLACK_CAL))
    return max(0.0, min(1.0, norm))

def on_line(x):
    return x <= ON_THRESH

def off_line(x):
    return x >= OFF_THRESH

class State:
    FOLLOW = 0
    SCAN_RIGHT = 1
    SCAN_LEFT = 2
    BACKUP = 3

state = State.FOLLOW
lost_since = None
scan_start = None

def set_forward():
    robot.forward(BASE_SPEED)

def set_spin_right():
    robot.right(TURN_SPEED)

def set_spin_left():
    robot.left(TURN_SPEED)

def stop():
    robot.stop()

try:
    set_forward()
    while True:
        x = read_reflectance_norm()
        avg.append(x)
        x_sm = sum(avg)/len(avg)

        if state == State.FOLLOW:
            # Simple “go straight”; you can optionally add tiny nudges here using last_seen_side
            if on_line(x_sm):
                # update last seen based on tiny derivative of reflectance if you have it,
                # or keep as-is when centered
                pass
            elif off_line(x_sm):
                # start debounce for "lost"
                if lost_since is None:
                    lost_since = time.time()
                elif time.time() - lost_since >= LOST_DEBOUNCE:
                    stop()
                    # bias first scan by last_seen_side
                    if last_seen_side >= 0:
                        state = State.SCAN_RIGHT
                    else:
                        state = State.SCAN_LEFT
                    scan_start = time.time()
            else:
                lost_since = None

        elif state in (State.SCAN_RIGHT, State.SCAN_LEFT):
            # spin slowly to re-acquire
            if state == State.SCAN_RIGHT:
                set_spin_right()
            else:
                set_spin_left()

            if on_line(x_sm):
                stop()
                # small pause helps stabilize readings
                time.sleep(0.05)
                set_forward()
                lost_since = None
                # remember direction we found it
                last_seen_side = +1 if state == State.SCAN_RIGHT else -1
                state = State.FOLLOW
                continue

            # if we’ve been scanning too long, swap direction or fallback
            if time.time() - scan_start > 1.2:  # ~1.2s per sweep
                stop()
                # swap side once; if already swapped, do a small backup
                if state == State.SCAN_RIGHT:
                    state = State.SCAN_LEFT
                else:
                    state = State.BACKUP
                scan_start = time.time()

        elif state == State.BACKUP:
            # back up a touch and re-try preferred side
            robot.backward(0.25)
            time.sleep(0.25)
            stop()
            state = State.SCAN_RIGHT if last_seen_side >= 0 else State.SCAN_LEFT
            scan_start = time.time()

        time.sleep(SAMPLE_DT)

except KeyboardInterrupt:
    stop()
