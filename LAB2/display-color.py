#!/usr/bin/env python3
import time
import board, busio
from PIL import Image, ImageDraw, ImageFont
from adafruit_ssd1306 import SSD1306_I2C
import adafruit_tcs34725

# --- config ---
WIDTH, HEIGHT = 128, 64
OLED_ADDR = 0x3C                  # OLED address (i2cdetect shows 3c)
UPDATE_HZ = 5                     # refresh rate

# --- devices ---
i2c  = busio.I2C(board.SCL, board.SDA)

# OLED (no reset pin; avoid GPIO4 conflicts)
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=OLED_ADDR)
oled.fill(0); oled.show()

# Color sensor
tcs = adafruit_tcs34725.TCS34725(i2c)   # default addr 0x29 (i2cdetect shows 29)
tcs.integration_time = 600              # ms
tcs.gain = 16                           # 1, 4, 16, 60

font = ImageFont.load_default()
period = 1.0 / UPDATE_HZ

while True:
    # Use gamma-corrected 0..255 values provided by the library
    r, g, b = tcs.color_rgb_bytes

    # Draw RGB values
    img = Image.new("1", (WIDTH, HEIGHT))
    d = ImageDraw.Draw(img)
    text_lines = [f"R: {r}", f"G: {g}", f"B: {b}"]

    # simple vertical layout, centered horizontally
    y = (HEIGHT - 3*12)//2  # ~12px per line with default font
    for line in text_lines:
        w, h = d.textbbox((0, 0), line, font=font)[2:]
        d.text(((WIDTH - w)//2, y), line, font=font, fill=255)
        y += 12

    oled.image(img)
    oled.show()
    time.sleep(period)
