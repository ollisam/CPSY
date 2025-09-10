import time
import board
import busio
import digitalio
import math
import adafruit_icm20x

# setup I2C communication
i2c = board.I2C()

# initialize the magnometer
mag = adafruit_icm20x.ICM20948(i2c, address=0x68)

red_led_pin = digitalio.DigitalInOut(board.D22)
red_led_pin.direction = digitalio.Direction.OUTPUT
red_led_pin.value = False

def calculate_heading(mag_x, mag_y):
    MX_OFFSET = 52.725
    MY_OFFSET = 126.825
    mx = mag_x - MX_OFFSET
    my = mag_y - MY_OFFSET
    heading = math.degrees(math.atan2(mx, my))
    return (heading + 360.0) % (360.0)

def is_north(mag_x, mag_y):
    heading = calculate_heading(mag_x, mag_y)
    threshold = 15  # degrees margin for north
    if heading <= threshold or heading >= (360 - threshold):
        return True
    else:
        return False

def compass_reading():
    while True:
        mag_x, mag_y, mag_z = mag.magnetic
        heading = calculate_heading(mag_x, mag_y)
        print(f"Magnometer X: {mag_x}, Y: {mag_y}, Z: {mag_z}")
        print(f"Heading: {heading:.2f} degrees")

        if is_north(mag_x, mag_y):
            red_led_pin.value = True
            print("Facing North! 🙂 LED ON")
        else:
            red_led_pin.value = False
            print("Not North. 🙁 LED OFF")

        time.sleep(1)

if __name__ == "__main__":
    compass_reading()
