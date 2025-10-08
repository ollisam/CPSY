import RPi.GPIO as GPIO
import time

pins = [13, 19, 12, 18]
GPIO.setmode(GPIO.BCM)
for p in pins:
    GPIO.setup(p, GPIO.OUT)

while True:
    for p in pins:
        print(f"Setting GPIO{p} HIGH")
        GPIO.output(p, 1)
        time.sleep(2)
        GPIO.output(p, 0)
