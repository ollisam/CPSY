import os
import time
import curses

from gpiozero import DigitalOutputDevice, Motor, Robot

try:
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)

    stdscr.addstr(0, 10, "Hit 'q' to quit")
    stdscr.nodelay(1)  # nodelay(1) give us a -1 back when nothing is pressed
    direction = None

    # Configure your own pins.
    robot = Robot(
   	 left=Motor('GPIO13', 'GPIO19'), # (forward, backwards)
   	 right=Motor('GPIO12', 'GPIO18')
    )

    # Here starts the code to make the robot move
    while direction != ord('q'):
        stdscr.refresh()
        direction = stdscr.getch()  # Gets the key which is pressed
        if direction == ord('e'):
            stdscr.addstr(1, 10, "Stop    ")
            robot.stop()
        if direction == ord('a'):
            stdscr.addstr(1, 10, "Left    ")
            robot.left()
        if direction == ord('s'):
            stdscr.addstr(1, 10, "Backward")
            robot.backward()
        if direction == ord('d'):
            stdscr.addstr(1, 10, "Right   ")
            robot.right()
        if direction == ord('w'):
            stdscr.addstr(1, 10, "Forward ")
            robot.forward()
        time.sleep(0.04)
finally:
    # Important to set everthing back by end of the script
    curses.nocbreak()
    stdscr.keypad(0)
    curses.endwin()
