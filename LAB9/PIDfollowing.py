from gpiozero import Robot
import time
import board
import busio
import adafruit_tcs34725

robot = Robot(left=('GPIO13','GPIO19'), right=('GPIO12','GPIO18'))

i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50
tcs.gain = 4


Kp = 24.0
Ki = 1.0
Kd = 1.0
base_speed = 60

middle_value = 3
error = 0
previos_error = 0
error_sum = 0

def setup():
    if tcs.begin()
        