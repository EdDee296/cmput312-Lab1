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

# Import task3 for dead reckoning controller
try:
    import task4
except ImportError:
    task4 = None
    print("Warning: task3.py not found. Dead reckoning controller will not be available.")

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
tank = MoveTank(OUTPUT_B, OUTPUT_C)
# wheel diam = 5.6cm (correct EV3 large motor tire diameter)

wheel_diameter_cm = 4.3
wheelbase_cm = 15.6  # Distance between wheels (should be actual wheelbase)


# ===== Path + timing =====
# half-size (lobe) ~ figure-eight scale
A_CM = 25.0
A = A_CM / 100.0                         # meters
# how fast we traverse theta(t)=k t (reduced for stability)
K_RAD_PER_S = 0.3
TOTAL_LOOPS = 2                          # how many figure-8 cycles to draw
# control timestep (s) - increased for stability
DT = 0.1
MAX_DPS = 360.0                          # safety clamp for motor command

left = LargeMotor(OUTPUT_B)
right = LargeMotor(OUTPUT_C)


def figure8_run():
    debug_print("=== Starting Figure-8 Movement ===")
    start = time()
    # Calculating the total time the robot will be traveling
    total_time = (2.0 * math.pi * TOTAL_LOOPS) / K_RAD_PER_S
    debug_print("Total time for figure-8: {:.2f} seconds".format(total_time))

    try:
        while True:
            t = time() - start
            if t >= total_time:
                break

            theta = K_RAD_PER_S * t
            s, c = math.sin(theta), math.cos(theta)
            c2 = math.cos(2.0 * theta)
            s2 = math.sin(2.0 * theta)

            # Calculate derivatives for figure-8 path
            xprime = -A * K_RAD_PER_S * s
            yprime = A * K_RAD_PER_S * c2

            xpprime = -A * (K_RAD_PER_S**2) * c
            ypprime = -2.0 * A * (K_RAD_PER_S**2) * s2

            # Calculate velocity and angular velocity
            v = math.hypot(xprime, yprime)  # m/s
            denom = (xprime*xprime + yprime*yprime)

            if denom < 1e-6:
                omega = 0.0
            else:
                omega = (xprime * ypprime - yprime * xpprime) / denom  # rad/s

            # Calculate left and right wheel velocities
            # v_r = v + (wheelbase/2) * omega, v_l = v - (wheelbase/2) * omega
            wheelbase_m = wheelbase_cm / 100.0  # Convert to meters
            v_r = v + 0.5 * wheelbase_m * omega
            v_l = v - 0.5 * wheelbase_m * omega

            # Convert to degrees per second for EV3 motors
            # v = (dps * π * diameter) / (180 * 2), so dps = (v * 180 * 2) / (π * diameter)
            wheel_circumference_m = (wheel_diameter_cm / 100.0) * math.pi
            dps_r = (v_r * 360.0) / wheel_circumference_m
            dps_l = (v_l * 360.0) / wheel_circumference_m

            # Clamp speeds to safe limits
            dps_r = max(-MAX_DPS, min(MAX_DPS, dps_r))
            dps_l = max(-MAX_DPS, min(MAX_DPS, dps_l))

            # Send commands to motors
            left.on(SpeedDPS(dps_l))
            right.on(SpeedDPS(dps_r))

            debug_print("t={:.2f}s, v={:.3f}m/s, ω={:.3f}rad/s, L={:.1f}dps, R={:.1f}dps".format(
                t, v, omega, dps_l, dps_r))

            sleep(DT)

    finally:
        left.off(brake=True)
        right.off(brake=True)
        debug_print("=== Figure-8 Complete ===")


def rectangle():
    # Calculate distance for rectangle: 5cm sides
    # Distance = 2*pi*r where r = wheel_radius, but we want actual distance
    # For 5cm distance: rotations = distance / wheel_circumference
    # EV3 large motor tire diameter in cm
    wheel_circumference_cm = math.pi * wheel_diameter_cm
    distance_cm = 5  # 5cm side length
    rotations = distance_cm / wheel_circumference_cm
    tank.on_for_rotations(SpeedPercent(50), SpeedPercent(
        50), rotations, brake=True, block=True)

    # Turn 90 degrees (4 times to make a rectangle)
    for i in range(4):
        tank.on_for_rotations(
            SpeedPercent(-50), SpeedPercent(50), 15.55/16.8, brake=True, block=True)
        time.sleep(1)
        tank.on_for_rotations(SpeedPercent(50), SpeedPercent(
            50), rotations, brake=True, block=True)
        time.sleep(1)


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
    figure8_run()
    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')

    # print something to the screen of the device
    print('Starting Lab 1...')

    # print something to the output panel in VS Code
    debug_print('Starting Lab 1...')

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
    """
    # rectangle()

    """
    TASK 4
    Run dead reckoning position controller from task4.py
    """
    # Uncomment the lines below to run dead reckoning:

    if task4:
        command_sequence = [
        [80, 60, 2],   # Row 1: 80% left, 60% right for 2 seconds
        [60, 60, 1],   # Row 2: 60% left, 60% right for 1 second
        [-50, 80, 2]   # Row 3: -50% left, 80% right for 2 seconds
    ]
        task4.dead_reckoning_position_controller(command_sequence)
    else:
        debug_print("Task 4 dead reckoning not available - task4.py not found")

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
