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

# ---------- LED (BCM pin D22) ----------
red_led_pin = digitalio.DigitalInOut(board.D22)
red_led_pin.direction = digitalio.Direction.OUTPUT
red_led_pin.value = False

# ---------- Calibration (hard-iron) ----------
def calibrate_magnetometer(duration_s=12, sleep_s=0.02):
    """
    Rotate the sensor slowly through all orientations (figure-8 in the air or
    full 360° on the table) during 'duration_s'. Returns (mx_off, my_off, mz_off).
    """
    print("Calibration started:")
    print(" • Keep the sensor away from the Pi/metal (>= 20–30 cm)")
    print(" • Move it slowly through many orientations for ~{}s…".format(duration_s))

    mx_min = my_min = mz_min =  1e9
    mx_max = my_max = mz_max = -1e9
    t0 = time.time()

    while time.time() - t0 < duration_s:
        try:
            mx, my, mz = icm.magnetic
        except Exception:
            # transient I2C hiccup; try next sample
            time.sleep(sleep_s)
            continue

        if mx < mx_min: mx_min = mx
        if mx > mx_max: mx_max = mx
        if my < my_min: my_min = my
        if my > my_max: my_max = my
        if mz < mz_min: mz_min = mz
        if mz > mz_max: mz_max = mz

        time.sleep(sleep_s)

    mx_off = (mx_max + mx_min) / 2.0
    my_off = (my_max + my_min) / 2.0
    mz_off = (mz_max + mz_min) / 2.0

    print("\nCalibration complete.")
    print(f"Offsets -> MX_OFFSET={mx_off:.3f}, MY_OFFSET={my_off:.3f}, MZ_OFFSET={mz_off:.3f}")
    print("Tip: Once you're happy, hard-code these offsets so you can skip calibration next run.\n")
    return mx_off, my_off, mz_off

# If your breakout's X/Y are rotated relative to how you're holding it,
# you can flip these booleans to match your "forward = north" intuition.
INVERT_X = False
INVERT_Y = False
SWAP_XY  = False  # set True if headings are ~90° off after calibration

def calculate_heading(mx, my, mx_off=0.0, my_off=0.0):
    # subtract offsets
    mx -= mx_off
    my -= my_off

    # optional axis adjustments
    if INVERT_X: mx = -mx
    if INVERT_Y: my = -my
    if SWAP_XY:  mx, my = my, mx

    # heading in degrees (0..360), assuming sensor is held level
    heading = math.degrees(math.atan2(my, mx))
    if heading < 0:
        heading += 360.0
    return heading

def is_north(heading_deg, threshold_deg=15):
    return (heading_deg <= threshold_deg) or (heading_deg >= (360 - threshold_deg))

def main():
    # --- Calibrate once at startup ---
    MX_OFFSET, MY_OFFSET, MZ_OFFSET = calibrate_magnetometer(duration_s=12)

    # --- Main loop ---
    try:
        while True:
            try:
                mx, my, mz = icm.magnetic
            except Exception:
                # transient read issue; LED off until next good sample
                red_led_pin.value = False
                time.sleep(0.05)
                continue

            heading = calculate_heading(mx, my, MX_OFFSET, MY_OFFSET)

            # Debug prints (comment out if you want it silent)
            print(f"Magnetometer X: {mx:.2f}, Y: {my:.2f}, Z: {mz:.2f}")
            print(f"Heading: {heading:.2f} degrees")

            # Configure logging once at program start
            logging.basicConfig(
                filename="magnetometer_10KOhm.log",  # or None for console only
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