from gpiozero import DigitalOutputDevice
import time

AIN1 = DigitalOutputDevice(20, initial_value=False)
AIN2 = DigitalOutputDevice(21, initial_value=False)

print("Forward (AIN1=1, AIN2=0)")
AIN1.on(); AIN2.off()
time.sleep(2)

print("Backward (AIN1=0, AIN2=1)")
AIN1.off(); AIN2.on()
time.sleep(2)

print("Stop (AIN1=0, AIN2=0)")
AIN1.off(); AIN2.off()
