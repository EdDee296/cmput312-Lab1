#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time
import math
#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_B, SpeedPercent, MoveTank, SpeedRPM, OUTPUT_C, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, GyroSensor
from ev3dev2.led import Leds

# Import task2 for error analysis functions
try:
    import task2
except ImportError:
    task2 = None
    print("Warning: task2.py not found. Error analysis functions will not be available.")

# TODO: Add code here
# state constants
ON = True
OFF = False
tank = MoveTank(OUTPUT_B, OUTPUT_C)
# wheel diam = 4.2cm
# B is a right motor, C is a left motor


def rectangle():
    # # long side of the rectangle
    # tank.on_for_rotations(
    #     SpeedPercent(-50), SpeedPercent(-50), 4, brake=True, block=True)
    # turn the vehicle by 90 degrees
    tank.on_for_rotations(SpeedPercent(
        -50), SpeedPercent(50), 15.55/16.8, brake=True, block=True)


    time.sleep(2)
    tank.on_for_rotations(SpeedPercent(
        -50), SpeedPercent(50), 15.55/16.8, brake=True, block=True)

    time.sleep(2)

    tank.on_for_rotations(SpeedPercent(
        -50), SpeedPercent(50), 15.55/16.8, brake=True, block=True)
    time.sleep(2)

    tank.on_for_rotations(SpeedPercent(
        -50), SpeedPercent(50), 15.55/16.8, brake=True, block=True)
    # # short side of the rectangle
    # tank.on_for_rotations(
    #     SpeedPercent(-50), SpeedPercent(-50), 2, brake=True, block=True)
    # # turn 90 degrees
    # tank.on_for_rotations(SpeedPercent(
    #     100), SpeedPercent(0), 1.5, brake=True, block=True)
    # # long side of the rectangle
    # tank.on_for_rotations(
    #     SpeedPercent(-50), SpeedPercent(-50), 4, brake=True, block=True)
    # # turn 90 degrees
    # tank.on_for_rotations(SpeedPercent(
    #     100), SpeedPercent(0), 1.5, brake=True, block=True)
    # # short side of the rectangle
    # tank.on_for_rotations(
    #     SpeedPercent(-50), SpeedPercent(-50), 2, brake=True, block=True)
    # # turn 90 degrees
    # tank.on_for_rotations(SpeedPercent(
    #     100), SpeedPercent(0), 1.5, brake=True, block=True)


def method1_encoder_based_error():
    """
    Method 1: Encoder-based straight line error measurement
    Measures deviation by comparing wheel rotations (no external sensors)
    """
    debug_print("=== Method 1: Encoder-Based Error Measurement ===")

    # Initialize motors
    left_motor = LargeMotor(OUTPUT_B)
    right_motor = LargeMotor(OUTPUT_C)

    # Reset encoder positions

    # Move forward for a set time

    # Track error over time
    errors = []
    start_time = time.time()

    while time.time() - start_time < 5:  # Run for 5 seconds
        left_pos = left_motor.position
        right_pos = right_motor.position

        # Calculate positional difference (error)
        position_error = abs(left_pos - right_pos)
        errors.append(position_error)

        debug_print("Left:", left_pos, "Right:",
                    right_pos, "Error:", position_error)
        time.sleep(0.1)

    # Stop motors
    left_motor.stop()
    right_motor.stop()

    # Calculate statistics
    avg_error = sum(errors) / len(errors)
    max_error = max(errors)
    final_error = errors[-1]

    debug_print("Method 1 Results:")
    debug_print("Average Error: {:.2f} degrees".format(avg_error))
    debug_print("Maximum Error: {:.2f} degrees".format(max_error))
    debug_print("Final Error: {:.2f} degrees".format(final_error))

    return {
        'method': 'Encoder-based',
        'avg_error': avg_error,
        'max_error': max_error,
        'final_error': final_error,
        'error_history': errors
    }


def method2_gyro_based_error():
    """
    Method 2: Gyro sensor-based straight line error measurement
    Uses gyroscope to measure angular deviation from intended direction
    """
    debug_print("=== Method 2: Gyro Sensor-Based Error Measurement ===")

    try:
        # Initialize gyro sensor (assumes connected to INPUT_3)
        gyro = GyroSensor(INPUT_3)
        gyro.mode = 'GYRO-ANG'  # Angle mode

        # Initialize motors
        tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

        # Reset gyro
        initial_angle = gyro.angle
        target_angle = initial_angle  # Should stay at this angle for straight line

        # Move forward
        tank_drive.on(SpeedPercent(50), SpeedPercent(50))

        # Track angular error over time
        angular_errors = []
        start_time = time.time()

        while time.time() - start_time < 5:  # Run for 5 seconds
            current_angle = gyro.angle
            angular_error = abs(current_angle - target_angle)
            angular_errors.append(angular_error)
            debug_print("Current Angle:", current_angle, "Target:",
                        target_angle, "Error:", angular_error)
            debug_print("Current Angle:", current_angle, "Target:",
                        target_angle, "Error:", angular_error)

            time.sleep(0.1)

        # Stop motors
        tank_drive.stop()

        # Calculate statistics
        avg_angular_error = sum(angular_errors) / len(angular_errors)
        max_angular_error = max(angular_errors)
        final_angular_error = angular_errors[-1]

        debug_print("Method 2 Results:")
        debug_print("Average Angular Error: {:.2f} degrees".format(
            avg_angular_error))
        debug_print("Maximum Angular Error: {:.2f} degrees".format(
            max_angular_error))
        debug_print("Final Angular Error: {:.2f} degrees".format(
            final_angular_error))

        return {
            'method': 'Gyro-based',
            'avg_error': avg_angular_error,
            'max_error': max_angular_error,
            'final_error': final_angular_error,
            'error_history': angular_errors
        }

    except Exception as e:
        debug_print("Gyro sensor error: {}".format(e))
        debug_print("Make sure gyro sensor is connected to INPUT_3")
        return None


def compare_methods(result1, result2):
    """
    Compare the two error measurement methods
    """
    debug_print("\n=== COMPARISON OF METHODS ===")

    if result2 is None:
        debug_print("Cannot compare - Gyro sensor method failed")
        return

    debug_print("\n1. {}:".format(result1['method']))
    debug_print("   - Measures: Wheel position differences")
    debug_print("   - Average Error: {:.2f}".format(result1['avg_error']))
    debug_print("   - Maximum Error: {:.2f}".format(result1['max_error']))
    debug_print("   - Final Error: {:.2f}".format(result1['final_error']))

    debug_print("\n2. {}:".format(result2['method']))
    debug_print("   - Measures: Angular deviation from straight path")
    debug_print("   - Average Error: {:.2f}°".format(result2['avg_error']))
    debug_print("   - Maximum Error: {:.2f}°".format(result2['max_error']))
    debug_print("   - Final Error: {:.2f}°".format(result2['final_error']))

    debug_print("\nFINDINGS:")
    debug_print("• Encoder method shows mechanical differences between wheels")
    debug_print("• Gyro method shows actual path deviation in space")
    debug_print("• Encoder errors accumulate over time due to wheel differences")
    debug_print("• Gyro method gives real-world orientation accuracy")
    debug_print(
        "• Both methods complement each other for complete error analysis")

    # Determine which method shows higher sensitivity
    if result1['avg_error'] > result2['avg_error']:
        debug_print(
            "• Encoder method detected higher average error (more sensitive to mechanical issues)")
    else:
        debug_print(
            "• Gyro method detected higher average error (more sensitive to path deviation)")


def straight_line_error_analysis():
    """
    Main function to run both error measurement methods and compare them
    """
    debug_print("Starting Straight Line Error Analysis...")
    debug_print(
        "This will test two different methods for measuring straight-line accuracy.\n")

    # Run Method 1: Encoder-based
    result1 = method1_encoder_based_error()

    time.sleep(2)  # Pause between tests

    # Run Method 2: Gyro-based
    result2 = method2_gyro_based_error()

    # Compare results
    compare_methods(result1, result2)

    return result1, result2


def method1_rotation_encoder_based_error():
    """
    Method 1: Encoder-based rotation error measurement
    Measures rotation accuracy by comparing expected vs actual wheel rotations
    """
    debug_print("=== Method 1: Encoder-Based Rotation Error Measurement ===")

    # Initialize motors
    left_motor = LargeMotor(OUTPUT_B)
    right_motor = LargeMotor(OUTPUT_C)

    # Reset encoder positions
    left_motor.position = 0
    right_motor.position = 0

    # Calculate expected rotation parameters
    target_rotation_degrees = 360  # One full rotation
    wheel_diameter_mm = 43  # EV3 Large tire diameter
    wheelbase_mm = 156  # Distance between wheels

    # FIXED FORMULA: For robot to rotate, each wheel travels arc length
    # Arc length = (robot_angle/360) * π * wheelbase
    arc_length_mm = (target_rotation_degrees / 360) * math.pi * wheelbase_mm

    # Convert to wheel rotations in degrees
    wheel_circumference_mm = math.pi * wheel_diameter_mm
    expected_wheel_degrees = (arc_length_mm / wheel_circumference_mm) * 360

    debug_print("Target robot rotation: {} degrees".format(
        target_rotation_degrees))
    debug_print("Wheelbase: {} mm".format(wheelbase_mm))
    debug_print("Wheel diameter: {} mm".format(wheel_diameter_mm))
    debug_print("Arc length per wheel: {:.2f} mm".format(arc_length_mm))
    debug_print("Wheel circumference: {:.2f} mm".format(
        wheel_circumference_mm))
    debug_print("Expected wheel rotation: {:.2f} degrees each".format(
        expected_wheel_degrees))

    # FIXED: Use consistent motor control for both wheels
    left_motor.on(SpeedPercent(30), block=False)   # Left wheel forward
    right_motor.on(SpeedPercent(-30), block=False)  # Right wheel backward

    # Track error over time
    rotation_errors = []
    start_time = time.time()
    run_time = 6  # Increased time to allow full rotation

    while time.time() - start_time < run_time:
        elapsed_time = time.time() - start_time
        left_pos = abs(left_motor.position)
        right_pos = abs(right_motor.position)

        # Average wheel rotation
        avg_wheel_rotation = (left_pos + right_pos) / 2

        # Calculate expected rotation at this time (linear progression)
        expected_rotation_now = expected_wheel_degrees * \
            (elapsed_time / run_time)
        rotation_error = abs(avg_wheel_rotation - expected_rotation_now)
        rotation_errors.append(rotation_error)

        debug_print("Time: {:.1f}s, Left: {:.1f}°, Right: {:.1f}°, Avg: {:.1f}°, Expected: {:.1f}°, Error: {:.2f}°".format(
            elapsed_time, left_pos, right_pos, avg_wheel_rotation, expected_rotation_now, rotation_error))

        # Stop if we've achieved expected rotation
        if avg_wheel_rotation >= expected_wheel_degrees:
            debug_print("Target wheel rotation achieved!")
            break

        time.sleep(0.2)

    # Stop motors
    left_motor.stop()
    right_motor.stop()

    # Calculate final statistics
    final_left = abs(left_motor.position)
    final_right = abs(right_motor.position)
    final_avg = (final_left + final_right) / 2
    final_error = abs(final_avg - expected_wheel_degrees)

    # Calculate actual robot rotation from wheel movement
    actual_arc_length = (final_avg / 360) * wheel_circumference_mm
    actual_robot_rotation = (
        actual_arc_length / (math.pi * wheelbase_mm)) * 360

    avg_error = sum(rotation_errors) / \
        len(rotation_errors) if rotation_errors else 0
    max_error = max(rotation_errors) if rotation_errors else 0

    debug_print("Method 1 Rotation Results:")
    debug_print("Expected wheel rotation: {:.2f}°".format(
        expected_wheel_degrees))
    debug_print("Actual average wheel rotation: {:.2f}°".format(final_avg))
    debug_print("Expected robot rotation: {:.2f}°".format(
        target_rotation_degrees))
    debug_print("Actual robot rotation: {:.2f}°".format(actual_robot_rotation))
    debug_print("Robot rotation error: {:.2f}°".format(
        abs(actual_robot_rotation - target_rotation_degrees)))
    debug_print("Average error: {:.2f}°".format(avg_error))
    debug_print("Maximum error: {:.2f}°".format(max_error))

    return {
        'method': 'Encoder-based Rotation',
        'expected_wheel_rotation': expected_wheel_degrees,
        'actual_wheel_rotation': final_avg,
        'expected_robot_rotation': target_rotation_degrees,
        'actual_robot_rotation': actual_robot_rotation,
        'final_error': abs(actual_robot_rotation - target_rotation_degrees),
        'avg_error': avg_error,
        'max_error': max_error,
        'error_history': rotation_errors
    }


def method2_rotation_gyro_based_error():
    """
    Method 2: Gyro sensor-based rotation error measurement
    Uses gyroscope to measure actual angular rotation vs intended rotation
    """
    debug_print("=== Method 2: Gyro Sensor-Based Rotation Error Measurement ===")

    try:
        # Initialize gyro sensor
        gyro = GyroSensor(INPUT_3)
        gyro.mode = 'GYRO-ANG'

        # Initialize motors
        tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)

        # Set target rotation
        target_rotation = 360  # degrees
        initial_angle = gyro.angle
        target_final_angle = initial_angle + target_rotation

        debug_print("Target rotation: {}°".format(target_rotation))
        debug_print("Initial angle: {}°".format(initial_angle))
        debug_print("Target final angle: {}°".format(target_final_angle))

        # Start rotation - FIXED: Use proper tank turn
        # Left forward, right backward
        tank_drive.on(SpeedPercent(30), SpeedPercent(-30))

        # Track angular error over time
        angular_errors = []
        start_time = time.time()
        run_time = 6  # Increased time to allow full rotation

        while time.time() - start_time < run_time:
            current_angle = gyro.angle
            elapsed_time = time.time() - start_time

            # Expected angle at this time (linear progression)
            expected_angle = initial_angle + \
                (target_rotation * elapsed_time / run_time)
            angular_error = abs(current_angle - expected_angle)
            angular_errors.append(angular_error)

            debug_print("Time: {:.1f}s, Current: {:.1f}°, Expected: {:.1f}°, Error: {:.2f}°".format(
                elapsed_time, current_angle, expected_angle, angular_error))

            # Check if we've completed the rotation
            actual_rotation_so_far = abs(current_angle - initial_angle)
            if actual_rotation_so_far >= target_rotation:
                debug_print("Target rotation achieved early!")
                break

            time.sleep(0.2)

        # Stop motors
        tank_drive.stop()

        # Final measurements
        final_angle = gyro.angle
        # Use absolute value
        actual_rotation = abs(final_angle - initial_angle)
        final_error = abs(actual_rotation - target_rotation)

        avg_error = sum(angular_errors) / len(angular_errors)
        max_error = max(angular_errors)

        debug_print("Method 2 Rotation Results:")
        debug_print("Target rotation: {}°".format(target_rotation))
        debug_print("Actual rotation: {:.2f}°".format(actual_rotation))
        debug_print("Final error: {:.2f}°".format(final_error))
        debug_print("Average error: {:.2f}°".format(avg_error))
        debug_print("Maximum error: {:.2f}°".format(max_error))

        return {
            'method': 'Gyro-based Rotation',
            'target_rotation': target_rotation,
            'actual_rotation': actual_rotation,
            'final_error': final_error,
            'avg_error': avg_error,
            'max_error': max_error,
            'error_history': angular_errors
        }

    except Exception as e:
        debug_print("Gyro sensor error: {}".format(e))
        debug_print("Make sure gyro sensor is connected to INPUT_3")
        return None


def compare_rotation_methods(result1, result2):
    """
    Compare the two rotation error measurement methods
    """
    debug_print("\n=== COMPARISON OF ROTATION METHODS ===")

    if result2 is None:
        debug_print("Cannot compare - Gyro sensor method failed")
        return

    debug_print("\n1. {}:".format(result1['method']))
    debug_print("   - Measures: Wheel encoder rotations vs calculated expected")
    debug_print(
        "   - Expected rotation: {:.2f}°".format(result1['expected_robot_rotation']))
    debug_print(
        "   - Actual rotation: {:.2f}°".format(result1['actual_robot_rotation']))
    debug_print("   - Final error: {:.2f}°".format(result1['final_error']))
    debug_print("   - Average error: {:.2f}°".format(result1['avg_error']))
    debug_print("   - Maximum error: {:.2f}°".format(result1['max_error']))

    debug_print("\n2. {}:".format(result2['method']))
    debug_print("   - Measures: Direct angular measurement from gyroscope")
    debug_print("   - Target rotation: {}°".format(result2['target_rotation']))
    debug_print(
        "   - Actual rotation: {:.2f}°".format(result2['actual_rotation']))
    debug_print("   - Final error: {:.2f}°".format(result2['final_error']))
    debug_print("   - Average error: {:.2f}°".format(result2['avg_error']))
    debug_print("   - Maximum error: {:.2f}°".format(result2['max_error']))

    debug_print("\nROTATION ANALYSIS FINDINGS:")
    debug_print(
        "• Encoder method depends on wheelbase and wheel diameter accuracy")
    debug_print("• Gyro method gives direct real-world rotation measurement")
    debug_print(
        "• Encoder method is affected by wheel slippage and surface friction")
    debug_print("• Gyro method is more accurate for navigation and orientation")
    debug_print(
        "• Encoder method shows mechanical inconsistencies in drive train")
    debug_print("• Both methods reveal different aspects of rotation accuracy")

    # Compare accuracy
    if result1['final_error'] < result2['final_error']:
        debug_print("• Encoder method showed lower final error")
    else:
        debug_print("• Gyro method showed lower final error")

    if result1['avg_error'] < result2['avg_error']:
        debug_print("• Encoder method showed more consistent performance")
    else:
        debug_print("• Gyro method showed more consistent performance")


def rotation_error_analysis():
    """
    Main function to run both rotation error measurement methods and compare them
    """
    debug_print("Starting Rotation Error Analysis...")
    debug_print(
        "This will test two different methods for measuring rotation accuracy.\n")

    # Run Method 1: Encoder-based rotation
    result1 = method1_rotation_encoder_based_error()

    time.sleep(2)  # Pause between tests

    # Run Method 2: Gyro-based rotation
    result2 = method2_rotation_gyro_based_error()

    # Compare results
    compare_rotation_methods(result1, result2)

    return result1, result2


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


def main():
    '''The main function of our program'''


    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')

    # print something to the screen of the device
    print('Hello World!')

    # print something to the output panel in VS Code
    debug_print('Hello VS Code!')

    #TASK 2
    # Run error analysis from task2.py
    # Uncomment the lines below to run error analysis:
    
    # if task2:
    #     task2.straight_line_error_analysis()
    #     time.sleep(3)  # Pause between analyses
    #     task2.rotation_error_analysis()
    # else:
    #     debug_print("Task 2 error analysis not available - task2.py not found")

    #TASK 3
    rectangle()

    # Original movement code (commented out)
    # tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    # tank_drive.on_for_seconds(SpeedPercent(-70), SpeedPercent(-50), 8)

    # wait a bit so you have time to look at the display before the program
    # exits
    time.sleep(5)


if __name__ == '__main__':
    main()
