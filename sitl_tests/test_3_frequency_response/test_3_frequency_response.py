###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Test 3 — Frequency Response / Bode Plot
#
#         Uses the Virtual Camera to close the feedback loop. The virtual target
#         oscillates sinusoidally at varying frequencies. For each frequency,
#         the drone's response is logged so you can compute gain & phase shift
#         to generate Bode plots for your thesis.
#
#         Output: graphs_generation/logs/test_3_bode_<axis>_<freq>Hz.csv
#                 (one CSV per frequency)
#
# Usage:  python sitl_tests/test_3_frequency_response.py
#         python sitl_tests/test_3_frequency_response.py --axis yaw --amplitude 3.0
###############################################################################

import argparse
import argparse
import math
import time
import sys
import os

# Add root directory to path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from sitl_tests.utils.virtual_camera import VirtualCamera
from sitl_tests.utils.sitl_utils import (
    load_config, sitl_connect, sitl_arm_and_takeoff,
    wait_for_position_data, CSVLogger,
)


# Default frequency sweep: logarithmically spaced from 0.05 Hz to 2 Hz
DEFAULT_FREQUENCIES = [0.05, 0.08, 0.1, 0.15, 0.2, 0.3, 0.5, 0.8, 1.0, 2.0]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test 3: Frequency Response — Sinusoidal target sweep for Bode plots"
    )
    parser.add_argument("--axis", type=str, default="yaw",
                        choices=["yaw", "altitude", "distance"],
                        help="Axis for the sinusoidal input (default: yaw)")
    parser.add_argument("--amplitude", type=float, default=3.0,
                        help="Amplitude of the oscillation (m or deg) (default: 3.0)")
    parser.add_argument("--initial-distance", type=float, default=None,
                        help="Initial distance to target in meters (default: 10.0)")
    parser.add_argument("--frequencies", type=str, default=None,
                        help="Comma-separated list of frequencies in Hz (default: logarithmic 0.05–2.0)")
    parser.add_argument("--duration-per-freq", type=float, default=30.0,
                        help="Recording duration per frequency in seconds (default: 30.0)")
    parser.add_argument("--settle-time", type=float, default=10.0,
                        help="Time to settle before each frequency test (s) (default: 10.0)")
    parser.add_argument("--takeoff-alt", type=float, default=10.0,
                        help="Takeoff altitude in meters (default: 10.0)")
    parser.add_argument("--loop-rate", type=float, default=20.0,
                        help="Control loop rate in Hz (default: 20.0)")
    parser.add_argument("--single-freq", type=float, default=None,
                        help="Run a single frequency instead of the full sweep")
    parser.add_argument("--run-name", type=str, default="auto",
                        help="Subfolder name for grouping logs. 'auto' creates run_001, run_002, etc.")
    return parser.parse_args()


def run_single_frequency(flight, config, vcam, target_area,
                         base_target_x, base_target_y, base_target_z,
                         freq_hz, amplitude, axis, duration, settle_time,
                         loop_rate, run_name):
    """
    Run a single frequency experiment: oscillate the target and record the response.

    Args:
        flight: Connected FlightController
        config: AppConfig
        vcam: VirtualCamera
        target_area: Target bounding box area for distance control
        base_target_x/y/z: Nominal target position (center of oscillation)
        freq_hz: Oscillation frequency in Hz
        amplitude: Oscillation amplitude in meters
        axis: Which axis to oscillate ('yaw', 'altitude', 'distance')
        duration: Recording duration in seconds
        settle_time: Time to settle on center target before starting oscillation
        loop_rate: Control loop rate in Hz
    """
    print(f"\n{'─' * 50}")
    print(f"  Frequency: {freq_hz} Hz | Amplitude: {amplitude}m | Axis: {axis}")
    print(f"  Duration: {duration}s + {settle_time}s settle")
    print(f"{'─' * 50}")

    # Prepare CSV
    csv_filename = os.path.join(run_name, f"test_3_bode_{axis}_{freq_hz:.3f}Hz.csv")
    logger = CSVLogger(csv_filename, [
        "time_s", "phase",
        "input_signal",  # The sinusoidal input (target offset in meters)
        "target_x", "target_y", "target_z",
        "drone_x", "drone_y", "drone_z", "drone_yaw_deg",
        "e_x", "e_y", "e_dist_m", "e_mag",
        "vx_cmd", "vz_cmd", "omega_z_cmd",
        "distance_to_target",
    ])

    # Load PD gains
    c = config.control
    k_p_yaw = c.k_p_yaw
    k_d_yaw = c.k_d_yaw
    k_p_vz = c.k_p_vz
    k_d_vz = c.k_d_vz
    k_p_vx = c.k_p_vx
    k_d_vx = c.k_d_vx
    r_stop = c.r_stop

    # PD state
    prev_e_x = 0.0
    prev_e_y = 0.0
    prev_e_dist = 0.0
    prev_time = time.time()

    loop_period = 1.0 / loop_rate
    total_time = settle_time + duration

    t_start = time.time()

    try:
        while True:
            t_now = time.time()
            t_elapsed = t_now - t_start

            if t_elapsed >= total_time:
                break

            # Calculate sinusoidal offset
            if t_elapsed >= settle_time:
                # Oscillation phase (time relative to oscillation start)
                t_osc = t_elapsed - settle_time
                input_signal = amplitude * math.sin(2.0 * math.pi * freq_hz * t_osc)
                phase = "SWEEP"
            else:
                input_signal = 0.0
                phase = "SETTLE"

            # Apply offset to the appropriate axis
            if axis == "yaw":
                # Move target laterally (right/starboard) relative to drone heading
                # To move laterally, we use perpendicular vector: (-sin(yaw), cos(yaw))
                att = flight.poll_attitude()
                yaw = att.yaw
                cur_target_x = base_target_x - input_signal * math.sin(yaw)
                cur_target_y = base_target_y + input_signal * math.cos(yaw)
                cur_target_z = base_target_z
            elif axis == "altitude":
                # Move target vertically (negative = up in NED)
                cur_target_x = base_target_x
                cur_target_y = base_target_y
                cur_target_z = base_target_z - input_signal
            elif axis == "distance":
                # Move target along the forward axis
                att = flight.poll_attitude()
                yaw = att.yaw
                cur_target_x = base_target_x + input_signal * math.cos(yaw)
                cur_target_y = base_target_y + input_signal * math.sin(yaw)
                cur_target_z = base_target_z

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

            # PD derivative
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
            e_scaled = min(1.0, e_mag / config.control.r_stop)
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
            if axis == "yaw":
                v_x = 0.0
                v_z = 0.0
            elif axis == "altitude":
                v_x = 0.0
                omega_z = 0.0
            elif axis == "distance":
                v_z = 0.0
                omega_z = 0.0

            # Send velocity command
            flight.send_velocity(v_x, 0.0, v_z, omega_z)

            # Log
            logger.log(
                f"{t_elapsed:.4f}", phase,
                f"{input_signal:.6f}",
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

            # Print progress every 1s
            if int(t_elapsed) != int(t_elapsed - loop_period):
                print(f"  t={t_elapsed:6.1f}s [{phase:6s}] f={freq_hz}Hz | "
                      f"input={input_signal:6.3f}m | e_x={e_x:7.4f} | ωz={omega_z:5.2f}")

            # Sleep to maintain loop rate
            t_sleep = loop_period - (time.time() - t_now)
            if t_sleep > 0:
                time.sleep(t_sleep)

    except KeyboardInterrupt:
        print("\n  Frequency test interrupted by user.")
        raise  # Re-raise to break the outer loop

    finally:
        flight.send_stop()
        logger.close()

    print(f"  Frequency {freq_hz} Hz complete → {logger.filepath}")
    return logger.filepath


def main():
    args = parse_args()

    # Determine frequency list
    if args.single_freq is not None:
        frequencies = [args.single_freq]
    elif args.frequencies is not None:
        frequencies = [float(f) for f in args.frequencies.split(",")]
    else:
        frequencies = DEFAULT_FREQUENCIES

    # Determine auto run_name
    if args.run_name == "auto":
        import glob
        logs_base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", args.axis)
        existing_runs = glob.glob(os.path.join(logs_base_dir, "run_*"))
        run_numbers = []
        for r in existing_runs:
            dirname = os.path.basename(r)
            try:
                num = int(dirname.split("_")[1])
                run_numbers.append(num)
            except (IndexError, ValueError):
                pass
        next_run = max(run_numbers) + 1 if run_numbers else 1
        args.run_name = f"run_{next_run:03d}"

    print("=" * 60)
    print("  TEST 3: FREQUENCY RESPONSE / BODE PLOT")
    print(f"  Axis: {args.axis} | Amplitude: {args.amplitude}m")
    print(f"  Run Directory: {args.run_name}")
    print(f"  Frequencies: {frequencies}")
    print(f"  Duration per frequency: {args.duration_per_freq}s + {args.settle_time}s settle")
    print(f"  Total estimated time: {len(frequencies) * (args.duration_per_freq + args.settle_time):.0f}s")
    print("=" * 60)

    # Load config and connect
    config = load_config()

    # Determine initial distance
    base_dist = args.initial_distance if args.initial_distance is not None else 10.0

    # Ensure the drone tries to hover at the base distance, so it doesn't saturate flying forward
    config.calibration.desired_stopping_distance_m = base_dist
    
    # Save test metadata to the run directory
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", args.axis, args.run_name)
    os.makedirs(logs_dir, exist_ok=True)
    with open(os.path.join(logs_dir, "metadata.txt"), "w") as f:
        f.write("TEST 3: FREQUENCY RESPONSE\n")
        f.write("=" * 30 + "\n")
        f.write(f"Axis: {args.axis}\n")
        f.write(f"Amplitude: {args.amplitude}\n")
        f.write(f"Initial Distance: {base_dist} m\n")
        f.write(f"Frequencies: {frequencies}\n")
        f.write("\nCONTROL GAINS (from config)\n")
        f.write("=" * 30 + "\n")
        f.write(f"k_p_vx: {config.control.k_p_vx}\n")
        f.write(f"k_d_vx: {config.control.k_d_vx}\n")
        f.write(f"dist_deadzone: {config.control.dist_deadzone}\n")
        f.write(f"k_p_yaw: {config.control.k_p_yaw}\n")
        f.write(f"k_p_vz: {config.control.k_p_vz}\n")
    

    flight = sitl_connect(config)

    # Arm and take off
    if not sitl_arm_and_takeoff(flight, target_alt=args.takeoff_alt):
        print("ERROR: Takeoff failed. Exiting.")
        return

    # Wait for position data
    if not wait_for_position_data(flight):
        print("ERROR: No position data. Exiting.")
        return


    # Create virtual camera
    vcam = VirtualCamera(
        hfov_deg=62.2,
        vfov_deg=48.8,
        optical_constant=config.calibration.optical_constant,
    )

    # Target area
    from classes.distance_estimator import DistanceEstimator
    dist_estimator = DistanceEstimator(config.calibration)
    target_area = dist_estimator.target_area(config.calibration.desired_stopping_distance_m)

    # Run each frequency
    output_files = []
    try:
        for i, freq in enumerate(frequencies):
            print(f"\n{'=' * 60}")
            print(f"  Frequency {i+1}/{len(frequencies)}: {freq} Hz")
            print(f"{'=' * 60}")

            # Get current position for target placement
            flight.poll_heartbeat()
            pos = flight.poll_local_position_ned()
            attitude = flight.poll_attitude()

            if pos is None:
                print("ERROR: Cannot read position for target placement. Exiting sweep.")
                break

            drone_yaw = attitude.yaw

            # Place base target in front of the drone based on its *current* position
            base_target_x = pos.x + base_dist * math.cos(drone_yaw)
            base_target_y = pos.y + base_dist * math.sin(drone_yaw)
            base_target_z = pos.z  # Same altitude

            print(f"  Drone starts at: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            print(f"  Base target at: ({base_target_x:.2f}, {base_target_y:.2f}, {base_target_z:.2f})")

            filepath = run_single_frequency(
                flight=flight,
                config=config,
                vcam=vcam,
                target_area=target_area,
                base_target_x=base_target_x,
                base_target_y=base_target_y,
                base_target_z=base_target_z,
                freq_hz=freq,
                amplitude=args.amplitude,
                axis=args.axis,
                duration=args.duration_per_freq,
                settle_time=args.settle_time,
                loop_rate=args.loop_rate,
                run_name=os.path.join(args.axis, args.run_name),
            )
            output_files.append(filepath)

            # Brief pause between frequencies to let the drone stabilize
            if i < len(frequencies) - 1:
                print("  Stabilizing for 5s before next frequency...")
                for _ in range(50):
                    flight.send_stop()
                    flight.poll_heartbeat()
                    time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nSweep interrupted by user.")
    finally:
        flight.send_stop()

    print(f"\n{'=' * 60}")
    print(f"  TEST 3 COMPLETE")
    print(f"  Output files:")
    for f in output_files:
        print(f"    • {f}")
    print(f"\n  To generate Bode plots, compute gain and phase from each CSV:")
    print(f"    gain(f) = amplitude_output / amplitude_input")
    print(f"    phase(f) = phase_output - phase_input")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
