import time
import board
import busio
import digitalio
import math
import adafruit_icm20x

# I2C setup
i2c = busio.I2C(board.SCL, board.SDA)
icm = adafruit_icm20x.ICM20948(i2c, address=0x68)

# LED setup
led = digitalio.DigitalInOut(board.D18)
led.direction = digitalio.Direction.OUTPUT

while True:
    # Magnetometer readings
    mag_x, mag_y, mag_z = icm.magnetic

    # Heading calculation
    heading = math.atan2(mag_y, mag_x) * (180 / math.pi)
    if heading < 0:
        heading += 360

    print(heading)

    # LED on if facing North (±15°)
    led.value = (heading <= 15 or heading >= 345)

    time.sleep(0.2)
