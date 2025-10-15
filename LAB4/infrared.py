import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Use the hardware chip select CE0 (pin 24 on Pi header, SPI0 CE0)
cs = digitalio.DigitalInOut(board.GPIO5)

# Create the MCP3008 object
mcp = MCP.MCP3008(spi, cs)

# Select channel 0 on the MCP3008
chan0 = AnalogIn(mcp, MCP.P0)

# Loop forever, print raw and voltage
while True:
    print("Raw ADC Value: ", chan0.value)
    print(f"ADC Voltage: {chan0.voltage:.3f} V")
    time.sleep(2)