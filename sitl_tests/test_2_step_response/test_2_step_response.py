###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Test 2 — Outer Loop Step Response
#
#         Uses the Virtual Camera to close the feedback loop. The PD controller
#         from config.yaml drives the drone toward a virtual target. At t=T_step,
#         the target is "teleported" to a new position (step input).
#
#         From the logged data, you can extract:
#           - Rise Time (10%→90% of final value)
#           - Overshoot (% above final value)
#           - Settling Time (within ±2% of final value)
#
#         Output: graphs_generation/logs/test_2_step_response_<axis>.csv
#
# Usage:  python sitl_tests/test_2_step_response.py
#         python sitl_tests/test_2_step_response.py --step-axis yaw --step-magnitude 5.0
###############################################################################

import argparse
import math
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from sitl_tests.utils.virtual_camera import VirtualCamera
from sitl_tests.utils.sitl_utils import (
    load_config, sitl_connect, sitl_arm_and_takeoff,
    wait_for_position_data, CSVLogger,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test 2: Outer Loop Step Response — PD controller with Virtual Camera"
    )
    parser.add_argument("--step-axis", type=str, default="yaw",
                        choices=["yaw", "alt", "dist"],
                        help="Axis for the step input (default: yaw)")
    parser.add_argument("--step-magnitude", type=float, default=3,
                        help="Step magnitude: meters for distance/altitude, meters offset for yaw (default: 3.0)")
    parser.add_argument("--initial-distance", type=float, default=None,
                        help="Initial distance to target in meters (default depends on axis: 0.6 for dist, 5.0 for others)")
    parser.add_argument("--settle-before", type=float, default=5.0,
                        help="Time to let PD settle on initial target before step (s) (default: 5.0)")
    parser.add_argument("--record-after", type=float, default=10.0,
                        help="Time to record after the step (s) (default: 10.0)")
    parser.add_argument("--takeoff-alt", type=float, default=10.0,
                        help="Takeoff altitude in meters (default: 10.0)")
    parser.add_argument("--loop-rate", type=float, default=20.0,
                        help="Control loop rate in Hz (default: 20.0)")
    return parser.parse_args()


def main():
    args = parse_args()
        
    if args.initial_distance is None:
        if args.step_axis == "dist":
            args.initial_distance = 0.6
            args.step_magnitude = 1.0
        else:
            args.initial_distance = 5.0

    print("=" * 60)
    print("  TEST 2: OUTER LOOP STEP RESPONSE")
    print(f"  Step axis: {args.step_axis} | Magnitude: {args.step_magnitude}")
    print(f"  Initial distance: {args.initial_distance}m")
    print("=" * 60)

    # Load config and connect
    config = load_config()
    flight = sitl_connect(config)

    # Arm and take off
    if not sitl_arm_and_takeoff(flight, target_alt=args.takeoff_alt):
        print("ERROR: Takeoff failed. Exiting.")
        return

    run_step_response(args, config, flight)

def run_step_response(args, config, flight, test_id=None):
    quiet = getattr(args, 'quiet', False)
    def qprint(*pargs, **kwargs):
        if not quiet:
            print(*pargs, **kwargs)

    # Wait for position data
    if not wait_for_position_data(flight):
        print("ERROR: No position data. Exiting.")
        return None, None

    # Get initial drone position to place the target relative to it
    flight.poll_heartbeat()
    pos = flight.poll_local_position_ned()
    attitude = flight.poll_attitude()

    if pos is None:
        print("ERROR: Cannot read initial position. Exiting.")
        return None, None

    drone_yaw = attitude.yaw

    # Place initial target in front of the drone at the specified distance
    target_x = pos.x + args.initial_distance * math.cos(drone_yaw)
    target_y = pos.y + args.initial_distance * math.sin(drone_yaw)
    target_z = pos.z  # Same altitude

    qprint(f"\nDrone position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), yaw={math.degrees(drone_yaw):.1f}°")
    qprint(f"Initial target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")

    if args.step_axis == "yaw":
        # Move target laterally (right/starboard) relative to drone heading
        step_target_x = target_x - args.step_magnitude * math.sin(drone_yaw)
        step_target_y = target_y + args.step_magnitude * math.cos(drone_yaw)
        step_target_z = target_z
    elif args.step_axis == "alt":
        # Move target vertically (Down is positive in NED, so negative = up)
        step_target_x = target_x
        step_target_y = target_y
        step_target_z = target_z - args.step_magnitude  # Move target UP
    elif args.step_axis == "dist":
        # Move target further away along the drone's heading
        step_target_x = target_x + args.step_magnitude * math.cos(drone_yaw)
        step_target_y = target_y + args.step_magnitude * math.sin(drone_yaw)
        step_target_z = target_z

    qprint(f"Stepped target: ({step_target_x:.2f}, {step_target_y:.2f}, {step_target_z:.2f})")

    # Target area for distance control (from calibration)
    from classes.distance_estimator import DistanceEstimator
    dist_estimator = DistanceEstimator(config.calibration)

    # Create virtual camera with calibrated values
    vcam = VirtualCamera(
        hfov_deg=62.2,
        vfov_deg=48.8,
        optical_constant=dist_estimator._optical_constant,
    )

    # Load PD gains from config
    c = config.control
    k_p_yaw = c.k_p_yaw
    k_d_yaw = c.k_d_yaw
    k_p_vz = c.k_p_vz
    k_d_vz = c.k_d_vz
    k_p_vx = c.k_p_vx
    k_d_vx = c.k_d_vx
    r_stop = c.r_stop

    target_area = dist_estimator.target_area(config.calibration.desired_stopping_distance_m)
    qprint(f"Target area (stopping distance {config.calibration.desired_stopping_distance_m}m): {target_area:.0f} px²")

    if test_id is None:
        # Determine run number for manual test
        import glob
        logs_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        existing_logs = glob.glob(os.path.join(logs_dir_path, f"test_2_{args.step_axis}_*.csv"))
        run_numbers = []
        for f in existing_logs:
            try:
                parts = f.split("_run_")
                if len(parts) > 1:
                    num = int(parts[-1].split(".")[0])
                    run_numbers.append(num)
            except ValueError:
                pass
        next_run = max(run_numbers) + 1 if run_numbers else 1
        run_id = f"run_{next_run:03d}"
        
        if args.step_axis == "dist":
            csv_filename = f"test_2_{args.step_axis}_p{k_p_vx:.2f}_d{k_d_vx:.4f}_{run_id}.csv"
        elif args.step_axis == "yaw":
            csv_filename = f"test_2_{args.step_axis}_p{k_p_yaw:.2f}_d{k_d_yaw:.4f}_{run_id}.csv"
        elif args.step_axis == "alt":
            csv_filename = f"test_2_{args.step_axis}_p{k_p_vz:.2f}_d{k_d_vz:.4f}_{run_id}.csv"
        else:
            csv_filename = f"test_2_{args.step_axis}_{run_id}.csv"
    else:
        # Autotuner is calling this
        csv_filename = f"autotune_outer_{args.step_axis}_{test_id}.csv"
    logger = CSVLogger(csv_filename, [
        "time_s", "phase",
        "target_x", "target_y", "target_z",
        "drone_x", "drone_y", "drone_z", "drone_yaw_deg",
        "e_x", "e_y", "e_dist_m", "e_mag",
        "vx_cmd", "vz_cmd", "omega_z_cmd",
        "distance_to_target",
    ])
    
    # Save metadata
    meta_filename = csv_filename.replace(".csv", "_metadata.txt")
    logs_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(logs_dir_path, exist_ok=True)
    with open(os.path.join(logs_dir_path, meta_filename), "w") as f:
        f.write("TEST 2: STEP RESPONSE\n")
        f.write("=" * 30 + "\n")
        f.write(f"Step Axis: {args.step_axis}\n")
        f.write(f"Magnitude: {args.step_magnitude}\n")
        f.write(f"Initial Distance: {args.initial_distance} m\n")
        f.write("\nCONTROL GAINS (from config)\n")
        f.write("=" * 30 + "\n")
        f.write(f"k_p_vx: {config.control.k_p_vx}\n")
        f.write(f"k_d_vx: {config.control.k_d_vx}\n")
        f.write(f"dist_deadzone: {config.control.dist_deadzone}\n")
        f.write(f"k_p_yaw: {config.control.k_p_yaw}\n")
        f.write(f"k_p_vz: {config.control.k_p_vz}\n")

    
    print(f"Starting test. Step will occur at T = {args.settle_before}s")

    # PD state
    prev_e_x = 0.0
    prev_e_y = 0.0
    prev_e_dist = 0.0
    prev_time = time.time()

    loop_period = 1.0 / args.loop_rate
    total_time = args.settle_before + args.record_after
    step_applied = False

    # Current target (starts at initial position)
    cur_target_x, cur_target_y, cur_target_z = target_x, target_y, target_z

    qprint(f"\nTimeline:")
    qprint(f"  [0s – {args.settle_before}s]  Settle on initial target")
    qprint(f"  [{args.settle_before}s]         STEP APPLIED")
    qprint(f"  [{args.settle_before}s – {total_time}s]  Record step response")
    qprint(f"  Total: {total_time}s\n")

    t_start = time.time()

    try:
        while True:
            t_now = time.time()
            t_elapsed = t_now - t_start

            if t_elapsed >= total_time:
                break

            # Apply step at the right time
            if t_elapsed >= args.settle_before and not step_applied:
                qprint(f"\n  >>> STEP APPLIED at t={t_elapsed:.2f}s <<<\n")
                cur_target_x, cur_target_y, cur_target_z = step_target_x, step_target_y, step_target_z
                step_applied = True

            phase = "STEP" if step_applied else "SETTLE"

            # Poll drone state
            flight.poll_heartbeat()
            pos = flight.poll_local_position_ned()
            attitude = flight.poll_attitude()

            if pos is None:
                time.sleep(loop_period)
                continue

            drone_yaw = attitude.yaw

            # Virtual camera projection
            cam_out = vcam.project(
                pos.x, pos.y, pos.z, drone_yaw,
                cur_target_x, cur_target_y, cur_target_z,
            )

            e_x = cam_out.e_x
            e_y = cam_out.e_y
            e_dist_m = cam_out.distance - config.calibration.desired_stopping_distance_m
            e_mag = min(1.0, math.sqrt(e_x**2 + e_y**2))

            # PD derivative calculation
            dt = t_now - prev_time
            if 0 < dt < config.control.max_derivative_dt:
                d_x = (e_x - prev_e_x) / dt
                d_y = (e_y - prev_e_y) / dt
                d_dist = (e_dist_m - prev_e_dist) / dt
            else:
                d_x = d_y = d_dist = 0.0

            # PD control output
            omega_z = k_p_yaw * e_x + k_d_yaw * d_x
            v_z = k_p_vz * e_y + k_d_vz * d_y
            v_x_request = k_p_vx * e_dist_m + k_d_vx * d_dist

            # Deadzones
            if abs(omega_z) < config.control.yaw_deadzone:
                omega_z = 0.0
            if abs(v_z) < config.control.vz_deadzone:
                v_z = 0.0
            if abs(e_dist_m) < config.control.dist_deadzone:
                v_x_request = 0.0

            # Velocity limits (coupled limiting for safety)
            e_scaled = min(1.0, e_mag / r_stop)
            if e_scaled >= 1.0:
                v_x_limit = 0.0
            else:
                v_x_limit = config.control.max_vx * (1 - e_scaled**2)

            if v_x_request > 0:
                v_x = min(v_x_request, v_x_limit)
            else:
                v_x = max(v_x_request, -v_x_limit)
                
            # Standard clipping for other axes
            v_z = max(min(v_z, config.control.max_vz), -config.control.max_vz)
            omega_z = max(min(omega_z, config.control.max_yaw_rate), -config.control.max_yaw_rate)

            # Isolate the test axis to avoid coupling dynamics
            if args.step_axis == "yaw":
                v_x = 0.0
                v_z = 0.0
            elif args.step_axis == "alt":
                v_x = 0.0
                omega_z = 0.0
            elif args.step_axis == "dist":
                v_z = 0.0
                omega_z = 0.0

            # Send velocity command
            flight.send_velocity(v_x, 0.0, v_z, omega_z)

            # Log data
            logger.log(
                f"{t_elapsed:.4f}", phase,
                f"{cur_target_x:.4f}", f"{cur_target_y:.4f}", f"{cur_target_z:.4f}",
                f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}", f"{math.degrees(drone_yaw):.2f}",
                f"{e_x:.6f}", f"{e_y:.6f}", f"{e_dist_m:.6f}", f"{e_mag:.6f}",
                f"{v_x:.4f}", f"{v_z:.4f}", f"{omega_z:.4f}",
                f"{cam_out.distance:.4f}",
            )

            # Update PD memory
            prev_e_x = e_x
            prev_e_y = e_y
            prev_e_dist = e_dist_m
            prev_time = t_now

            # Print progress every 0.5s
            if int(t_elapsed * 2) != int((t_elapsed - loop_period) * 2):
                qprint(f"  t={t_elapsed:6.2f}s [{phase:6s}] | e_x={e_x:7.4f} e_y={e_y:7.4f} "
                       f"e_dist={e_dist_m:7.4f} | vx={v_x:5.2f} vz={v_z:5.2f} ωz={omega_z:5.2f} | "
                       f"dist={cam_out.distance:.2f}m")

            # Sleep to maintain loop rate
            t_sleep = loop_period - (time.time() - t_now)
            if t_sleep > 0:
                time.sleep(t_sleep)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        qprint("\nSending stop command...")
        flight.send_stop()
        logger.close()

    qprint(f"\n{'=' * 60}")
    qprint(f"  TEST 2 STEP RESPONSE COMPLETE")
    qprint(f"  Data saved to: {logger.filepath}")
    qprint(f"{'=' * 60}")
    
    return logger.filepath, args.settle_before


if __name__ == "__main__":
    main()
