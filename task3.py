#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank, SpeedDPS
from time import sleep
import math
import time
import sys

# === Parameters ===
A_CM = 20.0                              # Amplitude in cm
A = A_CM / 100.0                         # Convert to meters
K_RAD_PER_S = 0.8                        # Angular frequency
TOTAL_LOOPS = 2                          # Number of figure-8 loops
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


# ===============================
# Rectangle movement
# ===============================
def rectangle():
    debug_print("=== Starting Rectangle Movement ===")

    # Side 1
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50), 3, brake=True)
    # Turn left 90°
    tank.on_for_rotations(SpeedPercent(-50), SpeedPercent(50),
                          15.55/16.8, brake=True)
    time.sleep(0.5)

    # Side 2
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50), 1, brake=True)
    tank.on_for_rotations(SpeedPercent(-50), SpeedPercent(50),
                          15.55/16.8, brake=True)
    time.sleep(0.5)

    # Side 3
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50), 3, brake=True)
    tank.on_for_rotations(SpeedPercent(-50), SpeedPercent(50),
                          15.55/16.8, brake=True)
    time.sleep(0.5)

    # Side 4
    tank.on_for_seconds(SpeedPercent(50), SpeedPercent(50), 1, brake=True)

    debug_print("=== Rectangle Complete ===")


# ===============================
# Parametric figure-8 (Lemniscate)
# ===============================
def figure8_run_fixed():
    debug_print("=== Starting FIXED Figure-8 Movement ===")
    start_time = time.time()

    wheelbase_m = wheelbase_cm / 100.0
    wheel_radius_m = (wheel_diameter_cm / 100.0) / 2.0

    a = A
    k = K_RAD_PER_S
    loops = TOTAL_LOOPS

    total_time = (2.0 * math.pi * loops) / k
    debug_print("Total trajectory time: {:.2f}s".format(total_time))

    v_min = 0.1  # prevent stalling
    step_count = 0
    next_step_time = start_time

    try:
        while True:
            elapsed_time = time.time() - start_time
            u = k * elapsed_time

            # Position (Lemniscate of Gerono)
            sin_u = math.sin(u)
            cos_u = math.cos(u)
            sin_2u = math.sin(2 * u)
            cos_2u = math.cos(2 * u)

            x = a * sin_u
            y = (a / 2.0) * sin_2u

            # Completion check
            if elapsed_time >= total_time:
                distance_from_start = math.sqrt(x*x + y*y)
                debug_print("Trajectory complete! u={:.2f}".format(u))
                debug_print("Final pos: x={:.3f}, y={:.3f}, dist={:.3f}m".format(
                    x, y, distance_from_start))
                break

            # Derivatives
            dx_dt = a * k * cos_u
            dy_dt = a * k * cos_2u
            d2x_dt2 = -a * (k**2) * sin_u
            d2y_dt2 = -2.0 * a * (k**2) * sin_2u

            # Linear velocity
            v_linear = math.sqrt(dx_dt*dx_dt + dy_dt*dy_dt)
            v_linear = max(v_linear, v_min)

            # Angular velocity (curvature method)
            speed_sq = dx_dt*dx_dt + dy_dt*dy_dt
            if speed_sq > 1e-8:
                curvature = (dx_dt*d2y_dt2 - dy_dt*d2x_dt2) / speed_sq
                omega = curvature
            else:
                omega = 0.0
            omega = max(-4.0, min(4.0, omega))

            # Wheel velocities
            v_left = v_linear - (omega * wheelbase_m) / 2.0
            v_right = v_linear + (omega * wheelbase_m) / 2.0

            circ = 2.0 * math.pi * wheel_radius_m
            dps_left = (v_left / circ) * 360.0
            dps_right = (v_right / circ) * 360.0

            max_speed = max(abs(dps_left), abs(dps_right))
            if max_speed > MAX_DPS and max_speed > 0:
                scale = MAX_DPS / max_speed
                dps_left *= scale
                dps_right *= scale

            left.on(SpeedDPS(dps_left))
            right.on(SpeedDPS(dps_right))

            if step_count % 20 == 0:
                debug_print("t={:.2f}s, u={:.2f}, pos=({:.3f},{:.3f})".format(
                    elapsed_time, u, x, y))
                debug_print(
                    "  v_lin={:.3f}, omega={:.3f}".format(v_linear, omega))
                debug_print("  dps_l={:.1f}, dps_r={:.1f}".format(
                    dps_left, dps_right))

            next_step_time += DT
            sleep_time = next_step_time - time.time()
            if sleep_time > 0:
                sleep(sleep_time)
            else:
                sleep(0.001)

            step_count += 1

    finally:
        left.off(brake=True)
        right.off(brake=True)
        debug_print("=== FIXED Figure-8 Complete ===")


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

    base_speed = 50
    turn_sensitivity = 2.0

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


# ===============================
# Main
# ===============================
def main():
    debug_print("=== TASK 3: RECTANGLE + FIGURE-8 ===")

    # rectangle()
    # time.sleep(3)

    # figure8_run_fixed()
    # OR
    figure8_run_waypoint()

    debug_print("=== TASK 3 COMPLETE ===")


if __name__ == '__main__':
    main()
