#!/usr/bin/env python3
'''Task 4: Dead Reckoning Position Controller'''

import os
import sys
import time
import math

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent


def debug_print(*args, **kwargs):
    '''Print to stderr and format for VS Code output panel.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def dead_reckoning_position_controller(command_sequence):
    """
    Dead Reckoning Position Controller

    Executes a sequence of motor commands and tracks robot position/orientation
    using differential drive kinematics and wheel odometry.

    Input: 3x3 array with [left_power, right_power, duration]
    Output: Final position (x, y) and orientation (theta) in degrees
    """
    debug_print("=== Dead Reckoning Position Controller ===")

    # Robot physical parameters (adjust based on your robot)
    wheel_diameter_mm = 43      # Wheel diameter in mm
    wheelbase_mm = 156          # Distance between wheels in mm
    wheel_circumference_mm = math.pi * wheel_diameter_mm

    # Command sequence: [left_power%, right_power%, duration_seconds]


    debug_print("Command sequence:")
    for i, cmd in enumerate(command_sequence):
        debug_print("  Row {}: Left={}%, Right={}%, Duration={}s".format(
            i+1, cmd[0], cmd[1], cmd[2]))

    # Initialize motors
    left_motor = LargeMotor(OUTPUT_B)
    right_motor = LargeMotor(OUTPUT_C)

    # Initialize robot state (starting at origin, facing forward)
    robot_x = 0.0           # X position in mm
    robot_y = 0.0           # Y position in mm
    robot_theta = 0.0       # Orientation in radians (0 = facing forward)

    # Track trajectory for analysis
    trajectory_log = []
    trajectory_log.append({
        'time': 0.0,
        'x': robot_x,
        'y': robot_y,
        'theta_deg': math.degrees(robot_theta),
        'command': 'START'
    })

    debug_print("\\nStarting position: ({:.2f}, {:.2f}) mm, orientation: {:.2f}°".format(
        robot_x, robot_y, math.degrees(robot_theta)))
    debug_print("Press ENTER when robot is aligned at (0,0) reference frame...")

    # Wait for user to align robot (in real scenario)
    # input()  # Uncomment for manual alignment

    total_time = 0.0

    # Execute each command in sequence
    for row_num, command in enumerate(command_sequence):
        left_power, right_power, duration = command

        debug_print("\\n--- Executing Row {} ---".format(row_num + 1))
        debug_print("Left motor: {}%, Right motor: {}%, Duration: {}s".format(
            left_power, right_power, duration))

        # Reset encoder positions for this segment
        left_motor.position = 0
        right_motor.position = 0

        # Start motors with specified powers
        left_motor.on(SpeedPercent(left_power), block=False)
        right_motor.on(SpeedPercent(right_power), block=False)

        # Track position during movement
        start_time = time.time()
        prev_left_pos = 0
        prev_right_pos = 0

        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            current_left_pos = left_motor.position
            current_right_pos = right_motor.position

            # Calculate incremental movement since last reading
            delta_left_pos = current_left_pos - prev_left_pos
            delta_right_pos = current_right_pos - prev_right_pos

            # Convert encoder degrees to distance (mm)
            delta_left_distance = (
                delta_left_pos / 360.0) * wheel_circumference_mm
            delta_right_distance = (
                delta_right_pos / 360.0) * wheel_circumference_mm

            # Calculate robot movement using differential drive kinematics
            delta_distance = (delta_left_distance + delta_right_distance) / 2.0
            delta_theta = (delta_right_distance -
                           delta_left_distance) / wheelbase_mm

            # Update robot position and orientation
            robot_x += delta_distance * math.sin(robot_theta + delta_theta/2)
            robot_y += delta_distance * math.cos(robot_theta + delta_theta/2)
            robot_theta += delta_theta

            # Normalize angle to [-π, π]
            robot_theta = math.atan2(
                math.sin(robot_theta), math.cos(robot_theta))

            # Log current state
            trajectory_log.append({
                'time': total_time + elapsed,
                'x': robot_x,
                'y': robot_y,
                'theta_deg': math.degrees(robot_theta),
                'command': 'Row {} (L:{}%, R:{}%)'.format(row_num+1, left_power, right_power)
            })

            prev_left_pos = current_left_pos
            prev_right_pos = current_right_pos
            time.sleep(0.1)  # Update at 10Hz

        # Stop motors after duration
        left_motor.stop()
        right_motor.stop()

        total_time += duration

        # Log final position for this command
        final_left_pos = left_motor.position
        final_right_pos = right_motor.position

        debug_print("Segment complete. Encoders: L={:.1f}°, R={:.1f}°".format(
            final_left_pos, final_right_pos))
        debug_print("Current position: ({:.2f}, {:.2f}) mm, orientation: {:.2f}°".format(
            robot_x, robot_y, math.degrees(robot_theta)))

        time.sleep(0.5)  # Brief pause between commands

    # Final results
    debug_print("\\n=== DEAD RECKONING RESULTS ===")
    debug_print("Final Position: ({:.2f}, {:.2f}) mm".format(robot_x, robot_y))
    debug_print("Final Orientation: {:.2f}° ({:.4f} radians)".format(
        math.degrees(robot_theta), robot_theta))
    debug_print("Total execution time: {:.1f} seconds".format(total_time))

    # Display trajectory summary
    debug_print("\\n=== TRAJECTORY LOG ===")
    debug_print("Time(s) | X(mm)   | Y(mm)   | Angle(°) | Command")
    debug_print("-" * 55)
    for point in trajectory_log[::5]:  # Show every 5th point to avoid clutter
        debug_print("{:6.1f}  | {:7.2f} | {:7.2f} | {:8.2f} | {}".format(
            point['time'], point['x'], point['y'], point['theta_deg'], point['command']))

    # Calculate total distance traveled
    total_distance = 0.0
    for i in range(1, len(trajectory_log)):
        dx = trajectory_log[i]['x'] - trajectory_log[i-1]['x']
        dy = trajectory_log[i]['y'] - trajectory_log[i-1]['y']
        total_distance += math.sqrt(dx*dx + dy*dy)

    debug_print("\\nTotal distance traveled: {:.2f} mm ({:.2f} cm)".format(
        total_distance, total_distance/10))

    # Display results on robot screen (if available)
    try:
        from ev3dev2.display import Display
        display = Display()
        display.clear()
        display.text_pixels("Dead Reckoning Results:", 0, 0)
        display.text_pixels("X: {:.1f}mm".format(robot_x), 0, 20)
        display.text_pixels("Y: {:.1f}mm".format(robot_y), 0, 40)
        display.text_pixels("Angle: {:.1f}deg".format(
            math.degrees(robot_theta)), 0, 60)
        display.text_pixels("Distance: {:.1f}mm".format(total_distance), 0, 80)
        display.update()
    except ImportError:
        debug_print("Display not available")

    return {
        'final_x': robot_x,
        'final_y': robot_y,
        'final_theta_rad': robot_theta,
        'final_theta_deg': math.degrees(robot_theta),
        'total_distance': total_distance,
        'total_time': total_time,
        'trajectory': trajectory_log
    }


def main():
    """
    Main function for Task 4: Dead Reckoning Position Controller
    """
    debug_print("=== TASK 4: DEAD RECKONING POSITION CONTROLLER ===")
    debug_print(
        "This program will execute a sequence of motor commands and track robot position.\\n")

    # Run the dead reckoning controller
    result = dead_reckoning_position_controller()

    debug_print("\\n=== TASK 4 COMPLETE ===")
    debug_print("Final position logged and displayed.")

    return result


if __name__ == '__main__':
    main()
