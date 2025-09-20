#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import math
#!/usr/bin/env python3

from time import sleep, time

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_B, SpeedPercent, MoveTank, SpeedRPM, OUTPUT_C, OUTPUT_C, SpeedDPS
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, GyroSensor
from ev3dev2.led import Leds

# Import task2 for error analysis functions
try:
    import task2
except ImportError:
    task2 = None
    print("Warning: task2.py not found. Error analysis functions will not be available.")

# Import task3 for rectangle and figure-8 movement
try:
    import task3
except ImportError:
    task3 = None
    print("Warning: task3.py not found. Rectangle and figure-8 movement will not be available.")

# Import task4 for dead reckoning controller
try:
    import task4
except ImportError:
    task4 = None
    print("Warning: task4.py not found. Dead reckoning controller will not be available.")

# Import task5 for Braitenberg vehicle
try:
    import task5
except ImportError:
    task5 = None
    print("Warning: task5.py not found. Braitenberg vehicle will not be available.")

# TODO: Add code here
# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print to stderr and format for VS Code output panel.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\\x1B[?25h', end='')
    else:
        print('\\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


def main():
    '''The main function of our program'''
    """
    # TASK 2
    # Run error analysis from task2.py
    """
    # Uncomment the lines below to run error analysis:

    # if task2:
    #     task2.straight_line_error_analysis()
    #     time.sleep(3)  # Pause between analyses
    #     task2.rotation_error_analysis()
    # else:
    #     debug_print("Task 2 error analysis not available - task2.py not found")

    """
    TASK 3
    Run rectangle and figure-8 movement from task3.py
    """
    if task3:
        task3.main()
    else:
        debug_print("Task 3 movement not available - task3.py not found")

    """
    TASK 4
    Run dead reckoning position controller from task4.py
    """
    # Uncomment the lines below to run dead reckoning:

    # if task4:
    #     task4.main()
    # else:
    #     debug_print("Task 4 dead reckoning not available - task4.py not found")

    """
    TASK 5
    Run Braitenberg vehicle behaviors from task5.py
    """
    # Uncomment the lines below to run Braitenberg vehicle:

    # if task5:
    #     task5.main()
    # else:
    #     debug_print(
    #         "Task 5 Braitenberg vehicle not available - task5.py not found")


if __name__ == '__main__':
    main()
