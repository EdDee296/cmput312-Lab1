#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank, SpeedDPS
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_3
from time import sleep
import math
import time
import sys

# === Parameters ===
A_CM = 20.0                              # Amplitude in cm
A = A_CM / 100.0                         # Convert to meters
K_RAD_PER_S = 0.8                        # Angular frequency
TOTAL_LOOPS = 1                          # Number of figure-8 loops
DT = 0.05                                # Control timestep
MAX_DPS = 400.0                          # Max motor speed (deg/s)

# Initialize motors
tank = MoveTank(OUTPUT_B, OUTPUT_C)
left = LargeMotor(OUTPUT_B)
right = LargeMotor(OUTPUT_C)

# Robot parameters
wheel_diameter_cm = 4.3
wheelbase_cm = 15.6  # Distance between wheels


def debug_print(*args, **kwargs):
    """Print to stderr for VS Code output panel."""
    print(*args, **kwargs, file=sys.stderr)


# Initialize gyro sensor (after debug_print is defined)
try:
    gyro = GyroSensor(INPUT_3)
    debug_print("Gyro sensor initialized")
except Exception as e:
    debug_print("Gyro sensor not found: {}".format(str(e)))
    gyro = None


# ===============================
# Rectangle movement
# ===============================
def rectangle():
    """Simple rectangle: 3s straight, turn 90°, 1s straight, turn 90°, repeat"""
    debug_print("=== Starting Rectangle Movement ===")

    # Define speeds
    straight_speed = 50
    turn_speed = 30

    # First long side (3 seconds)
    debug_print("Moving forward 3 seconds...")
    tank.on_for_seconds(SpeedPercent(straight_speed), SpeedPercent(straight_speed), 3, brake=True, block=True)

    # Turn 90 degrees
    debug_print("Turning 90 degrees...")
    tank.on_for_seconds(SpeedPercent(-turn_speed), SpeedPercent(turn_speed), 1.0, brake=True, block=True)

    # Short side (1 second)
    debug_print("Moving forward 1 second...")
    tank.on_for_seconds(SpeedPercent(straight_speed), SpeedPercent(straight_speed), 1, brake=True, block=True)

    # Turn 90 degrees
    debug_print("Turning 90 degrees...")
    tank.on_for_seconds(SpeedPercent(-turn_speed), SpeedPercent(turn_speed), 1.0, brake=True, block=True)

    # Second long side (3 seconds)
    debug_print("Moving forward 3 seconds...")
    tank.on_for_seconds(SpeedPercent(straight_speed), SpeedPercent(straight_speed), 3, brake=True, block=True)

    # Turn 90 degrees
    debug_print("Turning 90 degrees...")
    tank.on_for_seconds(SpeedPercent(-turn_speed), SpeedPercent(turn_speed), 1.0, brake=True, block=True)

    # Final short side (1 second)
    debug_print("Moving forward 1 second...")
    tank.on_for_seconds(SpeedPercent(straight_speed), SpeedPercent(straight_speed), 1, brake=True, block=True)

    # Final turn to complete rectangle
    debug_print("Final turn...")
    tank.on_for_seconds(SpeedPercent(-turn_speed), SpeedPercent(turn_speed), 1.0, brake=True, block=True)

    debug_print("=== Rectangle Complete ===")

def figure8_run():
    debug_print("=== Starting Figure-8 Movement ===")
    start = time.time()
    # Calculating the total time the robot will be traveling
    total_time = (2.0 * math.pi * TOTAL_LOOPS) / K_RAD_PER_S
    debug_print("Total time for figure-8: {:.2f} seconds".format(total_time))

    try:
        while True:
            t = time.time() - start
            if t >= total_time:
                break

            theta = K_RAD_PER_S * t
            s, c = math.sin(theta), math.cos(theta)
            c2 = math.cos(2.0 * theta)
            s2 = math.sin(2.0 * theta)

            # Calculate position (figure-8 lemniscate)
            x = A * s
            y = (A / 2.0) * s2

            # Calculate derivatives for figure-8 path
            xprime = A * K_RAD_PER_S * c
            yprime = A * K_RAD_PER_S * c2 * 2.0  # Factor of 2 from chain rule

            xpprime = -A * (K_RAD_PER_S**2) * s
            ypprime = -2.0 * A * (K_RAD_PER_S**2) * s2 * \
                2.0  # Factor of 2 from chain rule

            # Calculate velocity and angular velocity
            v = math.hypot(xprime, yprime)  # m/s
            v_squared = xprime*xprime + yprime*yprime

            if v_squared < 1e-6:
                omega = 0.0
            else:
                omega = (xprime * ypprime - yprime *
                         xpprime) / v_squared  # rad/s

            # Calculate left and right wheel velocities
            # v_r = v + (wheelbase/2) * omega, v_l = v - (wheelbase/2) * omega
            wheelbase_m = wheelbase_cm / 100.0  # Convert to meters
            v_r = v + 0.5 * wheelbase_m * omega
            v_l = v - 0.5 * wheelbase_m * omega

            # Convert to degrees per second for EV3 motors
            # v = ω * r, so ω = v/r (rad/s), then convert to deg/s
            wheel_radius_m = (wheel_diameter_cm / 100.0) / 2.0
            dps_r = (v_r / wheel_radius_m) * (180.0 / math.pi)
            dps_l = (v_l / wheel_radius_m) * (180.0 / math.pi)

            # Clamp speeds to safe limits
            dps_r = max(-MAX_DPS, min(MAX_DPS, dps_r))
            dps_l = max(-MAX_DPS, min(MAX_DPS, dps_l))

            # Send commands to motors
            left.on(SpeedDPS(dps_l))
            right.on(SpeedDPS(dps_r))

            debug_print("t={:.2f}s, pos=({:.3f},{:.3f}), v={:.3f}m/s, ω={:.3f}rad/s, L={:.1f}dps, R={:.1f}dps".format(
                t, x, y, v, omega, dps_l, dps_r))

            sleep(DT)

    finally:
        left.off(brake=True)
        right.off(brake=True)
        debug_print("=== Figure-8 Complete ===")

# ===============================
# Waypoint-based Figure-8
# ===============================


def resample_waypoints(points, spacing=0.03):
    """Resample waypoints at fixed spacing (m)."""
    if not points:
        return []
    resampled = [points[0]]
    acc = 0.0
    for i in range(1, len(points)):
        x0, y0 = resampled[-1]
        x1, y1 = points[i]
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-8:
            continue
        t = 0.0
        while acc + seg_len - t >= spacing:
            remain = spacing - acc
            frac = (t + remain) / seg_len
            nx = x0 + frac * (x1 - x0)
            ny = y0 + frac * (y1 - y0)
            resampled.append((nx, ny))
            t += remain
            acc = 0.0
        acc += seg_len - t
    return resampled


def figure8_run_waypoint():
    debug_print("=== Starting Waypoint Figure-8 ===")

    # Dense waypoints
    dense_points = []
    num_points = 500 * TOTAL_LOOPS
    for i in range(num_points + 1):
        u = (i / num_points) * (2.0 * math.pi * TOTAL_LOOPS)
        x = A * math.sin(u)
        y = (A / 2.0) * math.sin(2 * u)
        dense_points.append((x, y))

    # Resample
    waypoints = resample_waypoints(dense_points, spacing=0.03)
    debug_print("Resampled to {} waypoints".format(len(waypoints)))

    base_speed = 60
    turn_sensitivity = 2.5

    current_x, current_y = waypoints[0]
    if len(waypoints) > 1:
        tx, ty = waypoints[1]
        current_heading = math.atan2(ty - current_y, tx - current_x)
    else:
        current_heading = 0.0

    try:
        for i, (tx, ty) in enumerate(waypoints[1:]):
            dx = tx - current_x
            dy = ty - current_y
            dist = math.hypot(dx, dy)
            if dist < 0.01:
                continue

            target_heading = math.atan2(dy, dx)
            heading_error = target_heading - current_heading
            while heading_error > math.pi:
                heading_error -= 2*math.pi
            while heading_error < -math.pi:
                heading_error += 2*math.pi

            turn_adj = turn_sensitivity * heading_error * (180.0 / math.pi)
            turn_adj = max(-50, min(50, turn_adj))

            left_speed = max(-100, min(100, base_speed - turn_adj))
            right_speed = max(-100, min(100, base_speed + turn_adj))

            tank.on(SpeedPercent(left_speed), SpeedPercent(right_speed))

            move_time = min(dist / 0.05, 0.25)  # crude mapping
            sleep(move_time)

            current_x, current_y = tx, ty
            current_heading = target_heading

            if i % 10 == 0:
                debug_print("WP {}: ({:.3f},{:.3f}), dist={:.3f}".format(
                    i, tx, ty, dist))

    finally:
        tank.off(brake=True)
        debug_print("=== Waypoint Figure-8 Complete ===")


def figure8_run_2():
    debug_print("=== Starting Figure-8 Movement (Method 2) ===")
    start_time = time.time()
    total_time = (2.0 * math.pi * TOTAL_LOOPS) / K_RAD_PER_S
    debug_print("Total time for figure-8: {:.2f} seconds".format(total_time))

    try:
        while True:
            elapsed = time.time() - start_time
            u = K_RAD_PER_S * elapsed

            if elapsed >= total_time:
                break

            x = math.sin(u)
            y = math.sin(2*u)
            dx = math.cos(u)
            dy = 2*math.cos(2*u)
            ddx = -math.sin(u)
            ddy = -4*math.sin(2*u)

            v = math.sqrt(dx*dx + dy*dy)
            v_squared = dx*dx + dy*dy

            # Avoid division by zero
            if v_squared < 1e-6:
                dps_left = 0
                dps_right = 0
            else:
                # Calculate curvature-based wheel velocities
                omega = (ddy*dx - ddx*dy) / v_squared  # Angular velocity
                wheelbase_m = wheelbase_cm / 100.0

                vright = v + (wheelbase_m / 2.0) * omega
                vleft = v - (wheelbase_m / 2.0) * omega

                # Convert to degrees per second
                wheel_radius_m = (wheel_diameter_cm / 100.0) / 2.0
                dps_left_raw = (vleft / wheel_radius_m) * (180.0 / math.pi)
                dps_right_raw = (vright / wheel_radius_m) * (180.0 / math.pi)

                # Scale down the speeds to reasonable levels while preserving the ratio
                max_raw_speed = max(abs(dps_left_raw), abs(dps_right_raw))
                if max_raw_speed > MAX_DPS:
                    scale_factor = MAX_DPS / max_raw_speed
                    dps_left = dps_left_raw * scale_factor
                    dps_right = dps_right_raw * scale_factor
                else:
                    # Further scale down for smoother movement
                    scale_factor = 0.3  # Adjust this value to control overall speed
                    dps_left = dps_left_raw * scale_factor
                    dps_right = dps_right_raw * scale_factor

                # Final clamp to safe limits (should not be needed now)
                dps_left = max(-MAX_DPS, min(MAX_DPS, dps_left))
                dps_right = max(-MAX_DPS, min(MAX_DPS, dps_right))

            left.on(SpeedDPS(dps_left))
            right.on(SpeedDPS(dps_right))

            if int(elapsed * 10) % 10 == 0:  # Debug every 1 second
                debug_print("t={:.2f}s, v={:.3f}, ω={:.3f}, L={:.1f}dps, R={:.1f}dps".format(
                    elapsed, v, omega if v_squared >= 1e-6 else 0.0, dps_left, dps_right))

            sleep(DT)

    finally:
        left.off(brake=True)
        right.off(brake=True)
        debug_print("=== Figure-8 Complete (Method 2) ===")


def figure8_run_gyro_simple():
    """
    Simplified gyro-based figure-8 that doesn't separate turning and moving phases
    Uses continuous movement with heading correction
    """
    debug_print("=== Starting Simple Gyro-Based Figure-8 ===")

    if gyro is None:
        debug_print("ERROR: Gyro sensor not available! Using fallback method.")
        figure8_run_waypoint()
        return

    try:
        # Reset gyro
        gyro.reset()
        debug_print("Gyro sensor reset")
        sleep(1)

        # Generate waypoints with larger spacing
        dense_points = []
        num_points = 60 * TOTAL_LOOPS  # Even fewer points
        for i in range(num_points + 1):
            u = (i / num_points) * (2.0 * math.pi * TOTAL_LOOPS)
            x = A * math.sin(u)
            y = (A / 2.0) * math.sin(2 * u)
            dense_points.append((x, y))

        waypoints = resample_waypoints(
            dense_points, spacing=0.15)  # 15cm spacing
        debug_print("Generated {} waypoints".format(len(waypoints)))

        # Start from first waypoint
        current_x, current_y = waypoints[0]
        base_speed = 45

        for i, (target_x, target_y) in enumerate(waypoints[1:]):
            # Calculate direction to target
            dx = target_x - current_x
            dy = target_y - current_y
            target_distance = math.hypot(dx, dy)

            if target_distance < 0.05:  # Skip close waypoints
                continue

            target_heading_deg = math.atan2(dy, dx) * 180.0 / math.pi

            debug_print("WP {}: moving to ({:.3f}, {:.3f}), dist={:.3f}m".format(
                i+1, target_x, target_y, target_distance))

            # Move towards waypoint with continuous heading correction
            start_time = time.time()
            max_time = target_distance / 0.3 + 1.0  # Estimate time needed

            while time.time() - start_time < max_time:
                current_heading = gyro.angle
                heading_error = target_heading_deg - current_heading

                # Normalize heading error
                while heading_error > 180:
                    heading_error -= 360
                while heading_error < -180:
                    heading_error += 360

                # Calculate motor speeds with heading correction
                turn_correction = heading_error * 0.8  # Moderate correction
                turn_correction = max(-25, min(25, turn_correction))

                left_speed = base_speed - turn_correction
                right_speed = base_speed + turn_correction

                # Clamp speeds
                left_speed = max(-100, min(100, left_speed))
                right_speed = max(-100, min(100, right_speed))

                tank.on(SpeedPercent(left_speed), SpeedPercent(right_speed))
                sleep(0.1)

            # Brief stop at waypoint
            tank.off()
            sleep(0.1)

            # Update position
            current_x, current_y = target_x, target_y

    except Exception as e:
        debug_print("Error in simple gyro figure-8: {}".format(str(e)))
    finally:
        tank.off(brake=True)
        debug_print("=== Simple Gyro Figure-8 Complete ===")


def figure8_run_gyro_waypoint():
    """
    Gyro-based waypoint following for figure-8
    Uses gyroscope for accurate heading control and encoders for distance
    """
    debug_print("=== Starting Gyro-Based Waypoint Figure-8 ===")

    if gyro is None:
        debug_print("ERROR: Gyro sensor not available! Using fallback method.")
        figure8_run_waypoint()
        return

    try:
        # Reset and calibrate gyro
        gyro.reset()
        debug_print("Gyro sensor reset and calibrated")
        sleep(1)  # Allow calibration to settle

        # Generate dense waypoints for figure-8 path
        dense_points = []
        num_points = 100 * TOTAL_LOOPS  # Reduced from 300 to avoid too many waypoints
        for i in range(num_points + 1):
            u = (i / num_points) * (2.0 * math.pi * TOTAL_LOOPS)
            x = A * math.sin(u)
            y = (A / 2.0) * math.sin(2 * u)
            dense_points.append((x, y))

        # Resample waypoints at larger spacing to reduce turning
        waypoints = resample_waypoints(
            dense_points, spacing=0.10)  # Increased to 10cm spacing
        debug_print(
            "Generated {} waypoints for figure-8 path".format(len(waypoints)))

        # Robot state tracking
        current_x, current_y = waypoints[0]
        waypoint_tolerance = 0.05  # Increased tolerance

        # Control parameters - more lenient for smoother movement
        base_speed = 40  # Slightly faster
        turn_speed = 30
        heading_tolerance = 15.0  # Much more lenient heading tolerance

        # Robot physical parameters
        wheel_circumference_m = math.pi * (wheel_diameter_cm / 100.0)

        for i, (target_x, target_y) in enumerate(waypoints[1:]):
            if i % 15 == 0:  # Progress update
                debug_print("Moving to waypoint {}: ({:.3f}, {:.3f})".format(
                    i+1, target_x, target_y))

            # Calculate movement vector
            dx = target_x - current_x
            dy = target_y - current_y
            target_distance = math.hypot(dx, dy)

            if target_distance < 0.03:  # Skip very close waypoints - increased threshold
                debug_print("Skipping waypoint {}: too close ({:.3f}m)".format(
                    i+1, target_distance))
                current_x, current_y = target_x, target_y  # Update position
                continue

            # Calculate required heading (in degrees)
            target_heading_rad = math.atan2(dy, dx)
            target_heading_deg = target_heading_rad * 180.0 / math.pi

            debug_print("WP {}: target=({:.3f},{:.3f}), dist={:.3f}m, heading={:.1f}°".format(
                i+1, target_x, target_y, target_distance, target_heading_deg))

            # ==========================================
            # PHASE 1: TURN TO CORRECT HEADING
            # ==========================================
            current_gyro_angle = gyro.angle
            heading_error = target_heading_deg - current_gyro_angle

            # Normalize heading error to [-180, 180]
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360

            debug_print("Initial heading error: {:.1f}°".format(heading_error))

            # Only turn if heading error is significant
            if abs(heading_error) > heading_tolerance:
                # Turn until heading is correct
                turn_attempts = 0
                max_turn_attempts = 20  # Reduced attempts

                while abs(heading_error) > heading_tolerance and turn_attempts < max_turn_attempts:
                    # Determine turn direction and speed
                    if heading_error > 0:
                        # Turn left (counter-clockwise)
                        # Reduced multiplier
                        turn_power = min(turn_speed, abs(heading_error) * 1.0)
                        tank.on(SpeedPercent(-turn_power),
                                SpeedPercent(turn_power))
                    else:
                        # Turn right (clockwise)
                        # Reduced multiplier
                        turn_power = min(turn_speed, abs(heading_error) * 1.0)
                        tank.on(SpeedPercent(turn_power),
                                SpeedPercent(-turn_power))

                    sleep(0.15)  # Slightly longer turn time

                    # Update heading error
                    current_gyro_angle = gyro.angle
                    heading_error = target_heading_deg - current_gyro_angle

                    # Normalize heading error
                    while heading_error > 180:
                        heading_error -= 360
                    while heading_error < -180:
                        heading_error += 360

                    turn_attempts += 1
                    debug_print("Turn attempt {}: heading error now {:.1f}°".format(
                        turn_attempts, heading_error))

                # Stop turning
                tank.off()
                sleep(0.2)  # Longer pause after turning
            else:
                debug_print("Heading close enough, skipping turn phase")

            # ==========================================
            # PHASE 2: MOVE FORWARD TO WAYPOINT
            # ==========================================

            # Reset encoders for distance measurement
            left.position = 0
            right.position = 0

            # Calculate expected wheel rotation for target distance
            target_wheel_rotations = target_distance / wheel_circumference_m
            target_encoder_degrees = target_wheel_rotations * 360

            debug_print("Moving forward {:.3f}m (target encoder: {:.1f}°)".format(
                target_distance, target_encoder_degrees))

            # Move forward while monitoring distance
            movement_attempts = 0
            max_movement_attempts = 50  # Reduced attempts

            while movement_attempts < max_movement_attempts:
                # Check distance traveled using encoders
                avg_encoder_position = (
                    abs(left.position) + abs(right.position)) / 2
                distance_traveled = (
                    avg_encoder_position / 360) * wheel_circumference_m

                # Check if we've reached the waypoint
                if distance_traveled >= target_distance * 0.8:  # Reduced to 80% for more lenient stopping
                    debug_print("Reached waypoint! Distance traveled: {:.3f}m".format(
                        distance_traveled))
                    break

                # Simple forward movement with minimal heading correction
                current_gyro_angle = gyro.angle
                heading_error = target_heading_deg - current_gyro_angle

                # Normalize heading error
                while heading_error > 180:
                    heading_error -= 360
                while heading_error < -180:
                    heading_error += 360

                # Very small heading correction while moving
                correction = heading_error * 0.3  # Reduced correction factor
                # Reduced max correction
                correction = max(-5, min(5, correction))

                left_speed = base_speed - correction
                right_speed = base_speed + correction

                # Clamp speeds
                left_speed = max(-100, min(100, left_speed))
                right_speed = max(-100, min(100, right_speed))

                tank.on(SpeedPercent(left_speed), SpeedPercent(right_speed))

                sleep(0.1)  # Longer sleep for smoother movement
                movement_attempts += 1

                if movement_attempts % 10 == 0:  # Debug every 10th attempt
                    debug_print("Movement attempt {}: traveled {:.3f}m / {:.3f}m".format(
                        movement_attempts, distance_traveled, target_distance))

            # Stop at waypoint
            tank.off()
            sleep(0.1)

            # Update current position
            current_x, current_y = target_x, target_y

    except Exception as e:
        debug_print("Error in gyro-based figure-8: {}".format(str(e)))
        debug_print("Falling back to regular waypoint method")
        figure8_run_waypoint()

    finally:
        # Ensure motors are stopped
        tank.off(brake=True)
        debug_print("=== Gyro-Based Waypoint Figure-8 Complete ===")


# ===============================
# Main
# ===============================
def main():
    debug_print("=== TASK 3: RECTANGLE + FIGURE-8 ===")

    # rectangle()
    # time.sleep(3)
    rectangle()
    # Choose one of these figure-8 methods:
    # figure8_run()  # Original parametric method
    #figure8_run_2()  # Fixed alternative parametric method
    #figure8_run_waypoint()  # Time-based waypoint method
    # figure8_run_gyro_waypoint()  # Gyro-based waypoint method (improved)
    # figure8_run_gyro_simple()  # Simple continuous gyro method (try this first!)

    debug_print("=== TASK 3 COMPLETE ===")


if __name__ == '__main__':
    main()
