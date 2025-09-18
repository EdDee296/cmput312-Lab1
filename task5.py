#!/usr/bin/env python3
'''Task 5: Braitenberg Vehicle - Light-following behaviors'''

import os
import sys
import time
import math

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sensor.lego import ColorSensor


def debug_print(*args, **kwargs):
    '''Print to stderr and format for VS Code output panel.'''
    print(*args, **kwargs, file=sys.stderr)


class BraitenbergVehicle:
    """
    Braitenberg Vehicle implementation using two color sensors for light detection.

    The vehicle demonstrates four different behaviors:
    - Cowardice: Moves away from light
    - Aggression: Moves toward light aggressively
    - Love: Approaches light gently
    - Curiosity: Explores around light source
    """

    def __init__(self):
        # Initialize motors
        self.left_motor = LargeMotor(OUTPUT_B)
        self.right_motor = LargeMotor(OUTPUT_C)

        # Initialize color sensors for light detection
        self.left_sensor = ColorSensor(INPUT_1)
        self.right_sensor = ColorSensor(INPUT_4)

        # Set sensors to reflected light mode
        self.left_sensor.mode = 'COL-AMBIENT'
        self.right_sensor.mode = 'COL-AMBIENT'

        # Behavior parameters
        self.base_speed = 30  # Base motor speed
        self.max_speed = 80   # Maximum motor speed
        self.min_speed = -80  # Minimum motor speed (reverse)

        # Light detection parameters
        self.light_threshold = 0.1  # Minimum light level to react to
        self.max_light_value = 100  # Expected maximum light reading

        debug_print("Braitenberg Vehicle initialized")
        debug_print("Left sensor on INPUT_1, Right sensor on INPUT_4")

    def get_light_readings(self):
        """
        Get normalized light readings from both sensors.
        Returns values between 0 (dark) and 1 (bright).
        """
        try:
            # FIXED: ambient_light_intensity returns a single integer, not RGB tuple
            left_light = self.left_sensor.ambient_light_intensity
            right_light = self.right_sensor.ambient_light_intensity

            # Debug: Show raw values
            debug_print("Raw ambient light - Left: {}, Right: {}".format(left_light, right_light))

            # Normalize to 0-1 range (ambient light typically goes 0-100)
            left_normalized = left_light / 100.0
            right_normalized = right_light / 100.0

            return left_normalized, right_normalized

        except Exception as e:
            debug_print("Error reading sensors: {}".format(e))
            return 0.0, 0.0

    def apply_motor_speeds(self, left_speed, right_speed):
        """Apply speed limits and send commands to motors."""
        # Clamp speeds to safe limits
        left_speed = max(self.min_speed, min(self.max_speed, left_speed))
        right_speed = max(self.min_speed, min(self.max_speed, right_speed))

        # Apply speeds to motors
        self.left_motor.on(SpeedPercent(left_speed))
        self.right_motor.on(SpeedPercent(right_speed))

        return left_speed, right_speed

    def stop_motors(self):
        """Stop both motors."""
        self.left_motor.stop()
        self.right_motor.stop()

    def behavior_cowardice(self, left_light, right_light):
        """
        Cowardice behavior: Move away from light source.

        - If light below threshold: stay still (don't move)
        - If light detected: back away from it
        - If light on left: move right (turn right while backing)
        - If light on right: move left (turn left while backing)
        """
        avg_light = (left_light + right_light) / 2
        light_diff = left_light - right_light

        # If light is below threshold, stay still
        if avg_light < self.light_threshold:
            left_speed = 0
            right_speed = 0
            debug_print("Cowardice: No significant light detected - staying still")
        else:
            # Light detected - back away from it
            debug_print("Cowardice: Light detected - backing away")

            if abs(light_diff) > 0.05:  # Clear directional light
                if left_light > right_light:
                    # Light on LEFT → move RIGHT (turn right while backing)
                    left_speed = -self.base_speed - 20   # Left motor backward faster
                    right_speed = -self.base_speed + 10  # Right motor backward slower
                    debug_print("Cowardice: Light on left - turning right while backing")
                else:
                    # Light on RIGHT → move LEFT (turn left while backing)
                    left_speed = -self.base_speed + 10   # Left motor backward slower
                    right_speed = -self.base_speed - 20  # Right motor backward faster
                    debug_print("Cowardice: Light on right - turning left while backing")
            else:
                # Light straight ahead or evenly distributed - back straight away
                left_speed = -self.base_speed
                right_speed = -self.base_speed
                debug_print("Cowardice: Light ahead - backing straight away")

        return self.apply_motor_speeds(left_speed, right_speed)

    def behavior_aggression(self, left_light, right_light):
        """
        Aggression behavior: Move toward light source aggressively.

        - If light below threshold: stay still (don't move)
        - If light detected: aggressively approach the light
        - Faster and more aggressive than love behavior
        """
        avg_light = (left_light + right_light) / 2
        light_diff = left_light - right_light

        # If light is below threshold, stay still
        if avg_light < self.light_threshold:
            left_speed = 0
            right_speed = 0
            debug_print("Aggression: No significant light detected - staying still")
        else:
            # Light detected - approach it aggressively
            debug_print("Aggression: Light detected - attacking aggressively")

            if abs(light_diff) > 0.05:  # Clear directional light
                if left_light > right_light:
                    # Light on LEFT → turn LEFT aggressively (toward light)
                    left_speed = self.base_speed - 15   # Left motor slower
                    right_speed = self.base_speed + 25  # Right motor much faster
                    debug_print("Aggression: Light on left - turning left aggressively")
                else:
                    # Light on RIGHT → turn RIGHT aggressively (toward light)
                    left_speed = self.base_speed + 25   # Left motor much faster
                    right_speed = self.base_speed - 15  # Right motor slower
                    debug_print("Aggression: Light on right - turning right aggressively")
            else:
                # Light straight ahead - charge forward aggressively
                left_speed = self.base_speed + 20    # Faster than normal
                right_speed = self.base_speed + 20
                debug_print("Aggression: Light ahead - charging forward aggressively")

        return self.apply_motor_speeds(left_speed, right_speed)

    def behavior_love(self, left_light, right_light):
        """
        Love behavior: Approach light gently and stop when close.

        - If light below threshold: stay still (don't move)
        - If light detected: slowly approach the light
        - When very close: slow down even more or stop
        """
        avg_light = (left_light + right_light) / 2
        light_diff = left_light - right_light

        # If light is below threshold, stay still
        if avg_light < self.light_threshold:
            left_speed = 0
            right_speed = 0
            debug_print("Love: No significant light detected - staying still")
        else:
            # Light detected - approach it gently
            debug_print("Love: Light detected - approaching gently")

            # If very close to light (very bright), slow down or stop
            if avg_light > 0.8:
                left_speed = 5   # Very slow when close
                right_speed = 5
                debug_print("Love: Very close to light - moving very slowly")
            else:
                # Gentle approach with crossed wiring (like aggression but slower)
                if abs(light_diff) > 0.05:  # Clear directional light
                    if left_light > right_light:
                        # Light on LEFT → turn LEFT (toward light)
                        left_speed = self.base_speed - 10   # Left motor slower
                        right_speed = self.base_speed + 15  # Right motor faster
                        debug_print("Love: Light on left - turning left gently")
                    else:
                        # Light on RIGHT → turn RIGHT (toward light)
                        left_speed = self.base_speed + 15   # Left motor faster
                        right_speed = self.base_speed - 10  # Right motor slower
                        debug_print("Love: Light on right - turning right gently")
                else:
                    # Light straight ahead - approach slowly
                    left_speed = self.base_speed - 5    # Slightly slower than normal
                    right_speed = self.base_speed - 5
                    debug_print("Love: Light ahead - approaching slowly")

        return self.apply_motor_speeds(left_speed, right_speed)

    def behavior_curiosity(self, left_light, right_light):
        """
        Curiosity behavior: Explore around the light source.

        Approaches light but then circles around it instead of direct approach.
        """
        # Calculate light difference for turning behavior
        light_diff = left_light - right_light
        avg_light = (left_light + right_light) / 2.0

        # Base movement toward light
        approach_factor = avg_light * 20

        # Add circular motion when close to light
        if avg_light > 0.5:
            # Circle behavior - add differential to create turning
            turn_factor = 30 if light_diff > 0 else -30
            left_speed = self.base_speed + approach_factor + turn_factor
            right_speed = self.base_speed + approach_factor - turn_factor
        else:
            # Normal approach when far from light
            left_speed = self.base_speed + (right_light * 30)
            right_speed = self.base_speed + (left_light * 30)

        return self.apply_motor_speeds(left_speed, right_speed)

    def run_behavior(self, behavior_name, duration=30):
        """
        Run a specific behavior for a given duration.

        Args:
            behavior_name: 'cowardice', 'aggression', 'love', or 'curiosity'
            duration: Time in seconds to run the behavior
        """
        debug_print("\\n=== Starting {} Behavior ===".format(
            behavior_name.upper()))
        debug_print("Duration: {} seconds".format(duration))
        debug_print("Press any button to stop early")

        # Select behavior function
        behavior_functions = {
            'cowardice': self.behavior_cowardice,
            'aggression': self.behavior_aggression,
            'love': self.behavior_love,
            'curiosity': self.behavior_curiosity
        }

        if behavior_name not in behavior_functions:
            debug_print("Unknown behavior: {}".format(behavior_name))
            return

        behavior_func = behavior_functions[behavior_name]
        start_time = time.time()

        try:
            while time.time() - start_time < duration:
                # Get sensor readings
                left_light, right_light = self.get_light_readings()

                # Execute behavior
                left_speed, right_speed = behavior_func(
                    left_light, right_light)

                # Debug output every second
                if int(time.time() - start_time) % 1 == 0:
                    debug_print("Light: L={:.2f}, R={:.2f} | Motors: L={:.0f}, R={:.0f}".format(
                        left_light, right_light, left_speed, right_speed))

                time.sleep(0.1)  # 10Hz control loop

        finally:
            self.stop_motors()
            debug_print("=== {} Behavior Complete ===\\n".format(
                behavior_name.upper()))

    def demo_all_behaviors(self, duration_per_behavior=15):
        """
        Demonstrate all four Braitenberg behaviors in sequence.

        Args:
            duration_per_behavior: Duration in seconds for each behavior
        """
        debug_print("\\n=== BRAITENBERG VEHICLE DEMO ===")
        debug_print("Running all four behaviors automatically:")
        debug_print("1. Cowardice - Moves away from light")
        debug_print("2. Aggression - Moves toward light aggressively")
        debug_print("3. Love - Approaches light gently")
        debug_print("4. Curiosity - Explores around light")
        debug_print("Duration per behavior: {} seconds\\n".format(
            duration_per_behavior))

        behaviors = ['cowardice', 'aggression', 'love', 'curiosity']

        for i, behavior in enumerate(behaviors):
            debug_print(
                "\\n--- Behavior {} of 4: {} ---".format(i + 1, behavior.upper()))
            self.run_behavior(behavior, duration=duration_per_behavior)

            # Small pause between behaviors
            if i < len(behaviors) - 1:
                debug_print("Pausing for 2 seconds before next behavior...")
                time.sleep(2)

        debug_print("\\n=== ALL BEHAVIORS COMPLETE ===")


def run_braitenberg_behavior(behavior='aggression', duration=30):
    """
    Run a specific Braitenberg behavior without user interaction.

    Args:
        behavior: 'cowardice', 'aggression', 'love', 'curiosity', or 'demo'
        duration: Duration in seconds to run the behavior
    """
    debug_print("=== TASK 5: BRAITENBERG VEHICLE ===")
    debug_print("Behavior: {}".format(behavior.upper()))
    debug_print("Duration: {} seconds".format(duration))
    debug_print("Sensors: INPUT_1 (left), INPUT_4 (right)\\n")

    vehicle = BraitenbergVehicle()

    try:
        if behavior == 'demo':
            # Split time among 4 behaviors
            vehicle.demo_all_behaviors(duration_per_behavior=duration//4)
        elif behavior in ['cowardice', 'aggression', 'love', 'curiosity']:
            vehicle.run_behavior(behavior, duration=duration)
        else:
            debug_print("Unknown behavior: {}".format(behavior))
            debug_print(
                "Available behaviors: 'cowardice', 'aggression', 'love', 'curiosity', 'demo'")
            return False

    except KeyboardInterrupt:
        debug_print("\\nBehavior interrupted by user")
    except Exception as e:
        debug_print("Error: {}".format(e))
        return False
    finally:
        vehicle.stop_motors()

    return True


def main(behavior='aggression', duration=30):
    """
    Main function for Task 5: Braitenberg Vehicle

    Args:
        behavior: 'cowardice', 'aggression', 'love', 'curiosity', or 'demo'
        duration: Duration in seconds to run the behavior
    """
    debug_print("=== TASK 5: BRAITENBERG VEHICLE ===")
    debug_print("Light-following behaviors using color sensors")
    debug_print("Sensors: INPUT_1 (left), INPUT_4 (right)")

    try:
        # Run the specified behavior directly
        return run_braitenberg_behavior(behavior, duration)

    except KeyboardInterrupt:
        debug_print("\\nProgram interrupted by user")
        return False
    except Exception as e:
        debug_print("Error: {}".format(e))
        return False
    finally:
        # Ensure motors are stopped
        try:
            left_motor = LargeMotor(OUTPUT_B)
            right_motor = LargeMotor(OUTPUT_C)
            left_motor.stop()
            right_motor.stop()
        except:
            pass
        debug_print("\\n=== TASK 5 COMPLETE ===")


if __name__ == '__main__':
    main('love', 15)
