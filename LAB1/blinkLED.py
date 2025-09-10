import RPi.GPIO as GPIO
import time

ledPin = 22
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ledPin, GPIO.OUT)
GPIO.output(ledPin, GPIO.LOW)

try:
    GPIO.output(ledPin, GPIO.HIGH)
    time.sleep(1.0)
    GPIO.output(ledPin, GPIO.LOW)
    time.sleep(1.0)
except:
    GPIO.output(ledPin, GPIO.LOW)
    GPIO.cleanup()
