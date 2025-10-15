import time

# --- Motors ---
import curses
from gpiozero import DigitalOutputDevice, Motor, Robot

# --- IR distance via MCP3008 (from your infrared.py) ---
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# --- CONFIG ---
NEAR_VOLTAGE = 2.3      # volts considered "too close" (~20 cm)
SAMPLE_INTERVAL = 0.05  # seconds between sensor reads (50 ms)

def build_robot():
    return Robot(
        left = Motor('GPIO13', 'GPIO19'),   # (forward, backward)
        right = Motor('GPIO12', 'GPIO18')
    )

def build_ir_channel():
    # SPI + MCP3008 channel 0
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    cs = digitalio.DigitalInOut(board.D5)  # GPIO5
    mcp = MCP.MCP3008(spi, cs)
    return AnalogIn(mcp, MCP.P0)

def too_close(voltage: float) -> bool:
    """Return True if something is within our 'unsafe' range."""
    return voltage >= NEAR_VOLTAGE

def main():
    robot = build_robot()
    chan0 = build_ir_channel()

    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)
    stdscr.nodelay(1)

    stdscr.addstr(0, 2, "WASD to drive, 'e' stop, 'q' quit")
    stdscr.addstr(1, 2, "Failsafe: blocks/halts FORWARD if obstacle <~25 cm")

    moving_forward = False
    last_voltage = 0.0
    direction = -1 # no key pressed

    try:
        while direction != ord('q'):

            stdscr.refresh()
            direction = stdscr.getch()
            # --- Sensor handling ---
            last_voltage = chan0.voltage

            if not too_close(last_voltage):
                direction = ord('w')
                moving_forward = True

            # If already moving forward and we get too close: hard stop
            if moving_forward and too_close(last_voltage):
                robot.stop()
                moving_forward = False
                stdscr.addstr(1, 10, f"STOP: obstacle! V={last_voltage:0.2f}V      ")

            # --- Input handling ---
            if direction == ord('e'):
                robot.stop()
                moving_forward = False
                stdscr.addstr(1, 10, "Stop    ")

            if direction == ord('w'):
                if too_close(last_voltage):
                    robot.stop()
                    moving_forward = False
                    stdscr.addstr(1, 10, f"Blocked: too close (V={last_voltage:0.2f}V)   ")
                else:
                    robot.forward()
                    moving_forward = True
                    stdscr.addstr(1, 10, "Forward    ")

            if direction == ord('s'):
                robot.backward()
                moving_forward = False
                stdscr.addstr(1, 10, "Backward    ")

            if direction == ord('a'):
                robot.left()
                moving_forward = False
                stdscr.addstr(1, 10, "Left   ")

            if direction == ord('d'):
                robot.right()
                moving_forward = False
                stdscr.addstr(1, 10, "Right    ")

            # Display voltage
            time.sleep(0.04)

    finally:
        # Important to set everthing back by end of the script
        curses.nocbreak()
        stdscr.keypad(0)
        curses.endwin()
        robot.stop()

if __name__ == "__main__":
    main()
