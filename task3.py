#!/usr/bin/env python3
'''Task 3: Rectangle and Figure-8 Movement'''

import sys
import time
import math
from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank, SpeedDPS


def debug_print(*args, **kwargs):
    '''Print to stderr and format for VS Code output panel.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


# Initialize tank and motors
tank = MoveTank(OUTPUT_B, OUTPUT_C)
left = LargeMotor(OUTPUT_B)
right = LargeMotor(OUTPUT_C)

# Robot parameters
wheel_diameter_cm = 4.3
wheelbase_cm = 15.6  # Distance between wheels (should be actual wheelbase)

# Figure-8 parameters
A_CM = 25.0
A = A_CM / 100.0                         # meters
# how fast we traverse theta(t)=k t (reduced for stability)
K_RAD_PER_S = 0.3
TOTAL_LOOPS = 2                          # how many figure-8 cycles to draw
# control timestep (s) - increased for stability
DT = 0.1
MAX_DPS = 360.0                          # safety clamp for motor command


def rectangle():
    """
    Move the robot in a rectangular path with timed movements:
    - 3 seconds straight
    - Turn left
    - 1 second straight
    - Turn left
    - 3 seconds straight
    - Turn left
    - 1 second straight
    """
    debug_print("=== Starting Rectangle Movement ===")
    debug_print("Rectangle pattern: 3s → turn → 1s → turn → 3s → turn → 1s")

    # Side 1: Go straight for 3 seconds
    debug_print("  Side 1: Moving straight for 3 seconds")
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50),
                        3, brake=True, block=True)

    # Turn left 90 degrees
    debug_print("  Turn 1: Turning left 90 degrees")
    tank.on_for_rotations(SpeedPercent(-50), SpeedPercent(50),
                          15.55/16.8, brake=True, block=True)
    time.sleep(0.5)

    # Side 2: Go straight for 1 second
    debug_print("  Side 2: Moving straight for 1 second")
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50),
                        1, brake=True, block=True)

    # Turn left 90 degrees
    debug_print("  Turn 2: Turning left 90 degrees")
    tank.on_for_rotations(SpeedPercent(-50), SpeedPercent(50),
                          15.55/16.8, brake=True, block=True)
    time.sleep(0.5)

    # Side 3: Go straight for 3 seconds
    debug_print("  Side 3: Moving straight for 3 seconds")
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50),
                        3, brake=True, block=True)

    # Turn left 90 degrees
    debug_print("  Turn 3: Turning left 90 degrees")
    tank.on_for_rotations(SpeedPercent(-50), SpeedPercent(50),
                          15.55/16.8, brake=True, block=True)
    time.sleep(0.5)

    # Side 4: Go straight for 1 second
    debug_print("  Side 4: Moving straight for 1 second")
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50),
                        1, brake=True, block=True)

    debug_print("=== Rectangle Complete ===")


def figure8_run():
    """
    Move the robot in a figure-8 pattern using parametric equations
    """
    debug_print("=== Starting Figure-8 Movement ===")
    start = time.time()

    # Calculating the total time the robot will be traveling
    # one 8 run will take 2pi
    total_time = (2.0 * math.pi * TOTAL_LOOPS) / K_RAD_PER_S
    debug_print("Total time for figure-8: {:.2f} seconds".format(total_time))
    debug_print("Figure-8 parameters:")
    debug_print("  Amplitude: {}cm".format(A_CM))
    debug_print("  Angular velocity: {:.2f} rad/s".format(K_RAD_PER_S))
    debug_print("  Total loops: {}".format(TOTAL_LOOPS))

    try:
        while True:
            t = time.time() - start
            if t >= total_time:
                break

            # calculate theta
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


def main():
    """
    Main function for Task 3: Rectangle and Figure-8 Movement
    """
    debug_print("=== TASK 3: RECTANGLE AND FIGURE-8 MOVEMENT ===")

    # Run rectangle movement
    debug_print("\n--- Running Rectangle Movement ---")
    rectangle()

    # debug_print("\nWaiting 3 seconds before figure-8...")
    # time.sleep(3)

    # Run figure-8 movement
    # debug_print("\n--- Running Figure-8 Movement ---")
    # figure8_run()

    debug_print("\n=== TASK 3 COMPLETE ===")


if __name__ == '__main__':
    main()
