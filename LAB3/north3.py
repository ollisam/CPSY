import time
import math
import board
import busio
import digitalio
import adafruit_icm20x
import logging

# ---------- I2C + Sensor ----------
i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c, address=0x68)  # use 0x69 if AD0 is tied high

# ---------- LED (BCM pin D25) ----------
red_led_pin = digitalio.DigitalInOut(board.D25)
red_led_pin.direction = digitalio.Direction.OUTPUT
red_led_pin.value = False

def calculate_heading(mx, my):
    # MX_OFFSET=59.025, MY_OFFSET=114.150, MZ_OFFSET=-72.750

    # subtract offsets
    mx -= 59.025
    my -= 114.150

    # heading in degrees (0..360), assuming sensor is held level
    heading = math.degrees(math.atan2(my, mx))
    if heading < 0:
        heading += 360.0
    return heading

def is_north(heading_deg, threshold_deg=15):
    return (heading_deg <= threshold_deg) or (heading_deg >= (360 - threshold_deg))

def main():
    try:
        while True:
            try:
                mx, my, mz = icm.magnetic
            except Exception:
                # transient read issue; LED off until next good sample
                red_led_pin.value = False
                time.sleep(0.05)
                continue

            heading = calculate_heading(mx, my)

            # Debug prints
            print(f"Magnetometer X: {mx:.2f}, Y: {my:.2f}, Z: {mz:.2f}")
            print(f"Heading: {heading:.2f} degrees")

            # Configure logging once at program start
            logging.basicConfig(
                filename="magnetometer2.log",  # or None for console only
                level=logging.INFO,
                format="%(asctime)s - %(levelname)s - %(message)s"
            )

            # Then in your loop
            logging.info(f"Magnetometer X:{mx:.2f}, Y:{my:.2f}, Z:{mz:.2f}, Heading:{heading:.2f}")

            # LED on if facing roughly North (±15°)
            red_led_pin.value = is_north(heading, threshold_deg=15)
            print("Facing North! 🙂 LED ON" if red_led_pin.value else "Not North. 🙁 LED OFF")
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        red_led_pin.value = False

if __name__ == "__main__":
    main()
