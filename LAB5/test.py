from gpiozero import DigitalOutputDevice
import time

ain1 = DigitalOutputDevice(21)
ain2 = DigitalOutputDevice(20)

print("Forward...")
ain1.on(); ain2.off()
time.sleep(2)

print("Reverse...")
ain1.off(); ain2.on()
time.sleep(2)

print("Stop")
ain1.off(); ain2.off()
