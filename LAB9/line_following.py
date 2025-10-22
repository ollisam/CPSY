#!/usr/bin/env python3
import time
import math

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
BASE_SPEED = 0.15                # 0..1 (tune for your bot)
TURN_GAIN = 0.25                    # scales how hard to turn on color bias
MIN_STEER = 0.10                    # deadband: ignore very small biases
MAX_STEER = 0.6                     # clamp

# Color classification thresholds (on 0..255 gamma-corrected bytes)
GREEN_MIN = 40                      # require at least this much green to consider "green"
RED_MIN   = 40
DOMINANCE_MARGIN = 20               # G must exceed R by this (or R exceed G) to "win"
BLUE_SUPPRESS = 30                  # optional: if blue is high, ignore (helps on cyan/white)

# Safety / smoothing
SMOOTH_ALPHA = 0.35                 # EWMA on RGB for stability (0=off, 1=no smoothing)

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
    tcs.integration_time = 200              # ms
    tcs.gain = 60                          # 1, 4, 16, 60
    return oled, tcs

# =========================
# Helpers
# =========================
def too_close(voltage: float) -> bool:
    return voltage >= NEAR_VOLTAGE

def classify_color(r: float, g: float, b: float):
    """
    Return one of: 'green', 'red', or 'none'.
    Uses simple dominance logic with margins + basic blue suppression.
    """
    # optional suppress if blue is too high (helps ignore cyan/white LEDs/floor glare)
    if b > max(r, g) and b > BLUE_SUPPRESS:
        return 'none'

    if (g >= GREEN_MIN) and ((g - r) >= DOMINANCE_MARGIN) and (g >= b):
        return 'green'
    if (r >= RED_MIN) and ((r - g) >= DOMINANCE_MARGIN) and (r >= b):
        return 'red'
    return 'none'

def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))

def steer_from_color(tag: str) -> float:
    """
    Convert color tag to steering command in [-1, 1]
      negative => steer left, positive => steer right.
    """
    if tag == 'green':
        return -1.0
    if tag == 'red':
        return 1.0
    return 0.0

def mix_diff_drive(robot: Robot, base: float, steer: float):
    """
    Differential drive mixer.
    base in [0,1], steer in [-1,1] (left negative, right positive)
    """
    steer = max(-1.0, min(1.0, steer))
    left = clamp01(base * (1.0 - TURN_GAIN * max(0, steer)) * (1.0 + TURN_GAIN * max(0, -steer)))
    right = clamp01(base * (1.0 + TURN_GAIN * max(0, steer)) * (1.0 - TURN_GAIN * max(0, -steer)))

    # Normalize so one side stays at base and the other reduces, giving smooth nudge
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

def show_oled(oled, font, r, g, b, tag, v):
    img = Image.new("1", (WIDTH, HEIGHT))
    d = ImageDraw.Draw(img)
    lines = [
        f"R:{int(r)} G:{int(g)} B:{int(b)}",
        f"Seen: {tag}",
        f"IR V: {v:0.2f}V"
    ]
    # simple layout
    y = 6
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

    tick = 0
    try:
        while True:
            # --- read sensors ---
            r, g, b = tcs.color_rgb_bytes
            # EWMA smoothing
            r_s = SMOOTH_ALPHA * r + (1 - SMOOTH_ALPHA) * r_s
            g_s = SMOOTH_ALPHA * g + (1 - SMOOTH_ALPHA) * g_s
            b_s = SMOOTH_ALPHA * b + (1 - SMOOTH_ALPHA) * b_s

            voltage = chan0.voltage

            # --- obstacle check ---
            if too_close(voltage):
                # emergency stop + evade
                robot.stop()
                # quick back up
                robot.backward(BASE_SPEED * 0.8)
                time.sleep(BACKUP_TIME)
                robot.stop()
                # turn randomly-ish: bias right if red often means line on right; left if green
                # Here we peek at latest tag to decide
                tag_now = classify_color(r_s, g_s, b_s)
                if tag_now == 'green':
                    robot.left()
                else:
                    robot.right()
                time.sleep(EVADE_TURN_TIME)
                robot.stop()
                # continue loop (skip normal driving this tick)
                tick += 1
                time.sleep(SAMPLE_INTERVAL)
                if tick % OLED_EVERY_N_TICKS == 0:
                    show_oled(oled, font, r_s, g_s, b_s, f"BLOCK", voltage)
                continue

            # --- color -> steer ---
            tag = classify_color(r_s, g_s, b_s)
            steer_cmd = steer_from_color(tag)
            # deadband
            if abs(steer_cmd) < MIN_STEER:
                steer_cmd = 0.0

            # --- drive ---
            if steer_cmd == 0.0:
                robot.forward(BASE_SPEED)
            else:
                # gentle nudge while moving forward
                mix_diff_drive(robot, BASE_SPEED, steer_cmd)

            # --- OLED update ---
            tick += 1
            if tick % OLED_EVERY_N_TICKS == 0:
                show_oled(oled, font, r_s, g_s, b_s, tag, voltage)

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