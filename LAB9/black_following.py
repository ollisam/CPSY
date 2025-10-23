#!/usr/bin/env python3
import time
import math
import random

# --- Motor control ---
from gpiozero import Motor, Robot

# --- IR distance via MCP3008 ---
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# --- Color sensor + OLED ---
from PIL import Image, ImageDraw, ImageFont
from adafruit_ssd1306 import SSD1306_I2C
import adafruit_tcs34725

# =========================
# CONFIG (tune these)
# =========================
# OLED
WIDTH, HEIGHT = 128, 64
OLED_ADDR = 0x3C
UPDATE_HZ = 20                      # main loop rate (Hz)
OLED_EVERY_N_TICKS = 4              # update OLED every N loop ticks

# IR / Obstacle
NEAR_VOLTAGE = 2.0                  # >= means "too close"
SAMPLE_INTERVAL = 1.0 / UPDATE_HZ
BACKUP_TIME = 0.35                  # s: reverse if blocked
EVADE_TURN_TIME = 0.35              # s: turn after reversing

# Drive
BASE_SPEED = 0.18                   # 0..1 (tune for your bot)
SEARCH_SPEED = 0.15                 # turning speed while sweeping
TURN_GAIN = 0.10                    # (kept for mixer, not heavily used here)

# Line / color detection (BLACK line on lighter floor)
# We'll detect "black" when overall RGB brightness (0..765) is below this threshold.
BLACK_SUM_MAX = 120                 # <-- main threshold to tune (lower = stricter black)
HYSTERESIS = 15                     # avoid chatter around threshold
SMOOTH_ALPHA = 0.35                 # EWMA on RGB for stability (0=off, 1=no smoothing)

# Zig-zag search behavior when line is lost
SWEEP_BASE = 0.25                   # initial sweep duration (s)
SWEEP_INCREMENT = 0.12              # how much to extend each alternate sweep (s)
SWEEP_MAX = 1.2                     # clamp sweep duration (s)
PAUSE_BETWEEN_SWEEPS = 0.05         # brief pause between sweeps (s)

# =========================
# Hardware builders
# =========================
def build_robot():
    return Robot(
        left=Motor('GPIO13', 'GPIO19'),   # (forward, backward)
        right=Motor('GPIO12', 'GPIO18')
    )

def build_ir_channel():
    # SPI + MCP3008 channel 0
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    cs = digitalio.DigitalInOut(board.D5)  # GPIO5
    mcp = MCP.MCP3008(spi, cs)
    return AnalogIn(mcp, MCP.P0)

def build_oled_and_color():
    i2c = busio.I2C(board.SCL, board.SDA)

    # OLED
    oled = SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=OLED_ADDR)
    oled.fill(0); oled.show()

    # TCS34725 Color sensor
    tcs = adafruit_tcs34725.TCS34725(i2c)   # default 0x29
    tcs.integration_time = 600              # ms
    tcs.gain = 16                           # 1, 4, 16, 60
    return oled, tcs

# =========================
# Helpers
# =========================
def too_close(voltage: float) -> bool:
    return voltage >= NEAR_VOLTAGE

def brightness_sum(r: float, g: float, b: float) -> float:
    # 0..~765 range with rgb_bytes
    return float(r) + float(g) + float(b)

def is_on_black(sum_rgb: float, last_on_black: bool) -> bool:
    """
    Simple hysteresis: if we were on black, use a slightly higher threshold to stay on;
    if we were off, use the base threshold to require a clear black reading to switch on.
    """
    if last_on_black:
        return sum_rgb <= (BLACK_SUM_MAX + HYSTERESIS)
    else:
        return sum_rgb <= BLACK_SUM_MAX

def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))

def mix_diff_drive(robot: Robot, base: float, steer: float):
    """
    Differential drive mixer.
    base in [0,1], steer in [-1,1] (left negative, right positive)
    """
    steer = max(-1.0, min(1.0, steer))
    left = clamp01(base * (1.0 - TURN_GAIN * max(0, steer)) * (1.0 + TURN_GAIN * max(0, -steer)))
    right = clamp01(base * (1.0 + TURN_GAIN * max(0, steer)) * (1.0 - TURN_GAIN * max(0, -steer)))

    if steer < 0:   # left turn => slow left wheel
        left = clamp01(base * (1.0 - TURN_GAIN * abs(steer)))
        right = base
    elif steer > 0: # right turn => slow right wheel
        left = base
        right = clamp01(base * (1.0 - TURN_GAIN * abs(steer)))
    else:
        left = right = base

    robot.left_motor.forward(left)
    robot.right_motor.forward(right)

def show_oled(oled, font, r, g, b, state, v, ssum):
    img = Image.new("1", (WIDTH, HEIGHT))
    d = ImageDraw.Draw(img)
    lines = [
        f"R:{int(r)} G:{int(g)} B:{int(b)}",
        f"Sum:{int(ssum)} Th:{BLACK_SUM_MAX}",
        f"State: {state}",
        f"IR V: {v:0.2f}V"
    ]
    y = 2
    for line in lines:
        w, h = d.textbbox((0, 0), line, font=font)[2:]
        d.text(((WIDTH - w)//2, y), line, fill=255)
        y += 14
    oled.image(img)
    oled.show()

# =========================
# Main
# =========================
def main():
    robot = build_robot()
    chan0 = build_ir_channel()
    oled, tcs = build_oled_and_color()
    font = ImageFont.load_default()

    # EWMA init with first read
    r, g, b = tcs.color_rgb_bytes
    r_s, g_s, b_s = float(r), float(g), float(b)

    # --- Zig-zag state ---
    STATE_ON_LINE = "ON_LINE"
    STATE_LOST = "LOST"
    state = STATE_ON_LINE
    last_on_black = False

    sweep_dir = -1                  # -1 = left, +1 = right; start left
    sweep_time = SWEEP_BASE
    sweep_end_t = None

    tick = 0

    try:
        while True:
            # --- read sensors ---
            r, g, b = tcs.color_rgb_bytes
            # EWMA smoothing
            r_s = SMOOTH_ALPHA * r + (1 - SMOOTH_ALPHA) * r_s
            g_s = SMOOTH_ALPHA * g + (1 - SMOOTH_ALPHA) * g_s
            b_s = SMOOTH_ALPHA * b + (1 - SMOOTH_ALPHA) * b_s

            ssum = brightness_sum(r_s, g_s, b_s)
            on_black = is_on_black(ssum, last_on_black)
            last_on_black = on_black

            voltage = chan0.voltage

            # --- obstacle check ---
            if too_close(voltage):
                # emergency stop + evade
                robot.stop()
                robot.backward(BASE_SPEED * 0.8)
                time.sleep(BACKUP_TIME)
                robot.stop()
                # quick bias turn to clear obstacle
                # If we were searching, flip direction to avoid looping into obstacle
                if state == STATE_LOST:
                    sweep_dir *= -1
                if sweep_dir < 0:
                    robot.left(SEARCH_SPEED)
                else:
                    robot.right(SEARCH_SPEED)
                time.sleep(EVADE_TURN_TIME)
                robot.stop()

                # OLED tick for obstacle
                tick += 1
                time.sleep(SAMPLE_INTERVAL)
                if tick % OLED_EVERY_N_TICKS == 0:
                    show_oled(oled, font, r_s, g_s, b_s, "BLOCK", voltage, ssum)
                continue

            # --- line following / zig-zag search ---
            now = time.monotonic()

            if on_black:
                # We are on the line: go forward and reset search
                robot.forward(BASE_SPEED)
                state = STATE_ON_LINE
                sweep_time = SWEEP_BASE
                sweep_end_t = None
            else:
                # Lost the line: sweep left/right, increasing sweep time gradually
                if state != STATE_LOST:
                    state = STATE_LOST
                    sweep_dir = -1 if random.random() < 0.5 else 1
                    sweep_time = SWEEP_BASE
                    sweep_end_t = now + sweep_time
                    robot.left(SEARCH_SPEED) if sweep_dir < 0 else robot.right(SEARCH_SPEED)
                else:
                    # still lost — keep sweeping, flip direction after each interval
                    if sweep_end_t is None:
                        sweep_end_t = now + sweep_time
                        robot.left(SEARCH_SPEED) if sweep_dir < 0 else robot.right(SEARCH_SPEED)
                    elif now >= sweep_end_t:
                        # brief pause before switching direction
                        robot.stop()
                        time.sleep(PAUSE_BETWEEN_SWEEPS)
                        # extend the sweep a bit (capped)
                        sweep_time = min(SWEEP_MAX, sweep_time + SWEEP_INCREMENT)
                        sweep_dir *= -1
                        sweep_end_t = time.monotonic() + sweep_time
                        robot.left(SEARCH_SPEED) if sweep_dir < 0 else robot.right(SEARCH_SPEED)

            # --- OLED update ---
            tick += 1
            if tick % OLED_EVERY_N_TICKS == 0:
                show_oled(oled, font, r_s, g_s, b_s, state, voltage, ssum)

            time.sleep(SAMPLE_INTERVAL)

    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        # clear OLED
        try:
            oled.fill(0); oled.show()
        except Exception:
            pass


if __name__ == "__main__":
    main()