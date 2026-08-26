###############################################################################
# Author: Luca Boninsegna
# Date:   26/08/2026
# Descr:  Test 4 — Trajectory Scenario Simulation
#
#         Uses the Virtual Camera (with optional noise injection) to close the
#         feedback loop against ArduPilot SITL. A virtual target follows a
#         predefined walking trajectory, and the PD controller (identical to
#         mission_controller.py) tracks it with ALL 3 AXES ACTIVE simultaneously.
#
#         Unlike Test 2 (step response) and Test 3 (frequency response) where
#         axes are isolated, this test captures real coupling dynamics:
#           - Does yawing disturb altitude?
#           - Does forward acceleration cause yaw drift?
#           - How does detection noise affect tracking smoothness?
#
#         Supports running with ideal (clean) camera or noisy camera for A/B
#         comparison of vision imperfection effects.
#
#         Output: sitl_tests/test_4_trajectory/logs/<run>/test_4_<scenario>_<mode>.csv
#
# Usage:  python sitl_tests/test_4_trajectory/test_4_trajectory.py
#         python sitl_tests/test_4_trajectory/test_4_trajectory.py --scenario circle --noise
#         python sitl_tests/test_4_trajectory/test_4_trajectory.py --scenario all --compare
###############################################################################

import argparse
import glob
import math
import time
import sys
import os

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from sitl_tests.utils.virtual_camera import VirtualCamera
from sitl_tests.utils.noisy_camera import NoisyVirtualCamera, create_ideal_camera
from sitl_tests.utils.sitl_utils import (
    load_config, sitl_connect, sitl_arm_and_takeoff,
    wait_for_position_data, CSVLogger,
)
from sitl_tests.test_4_trajectory.trajectories import (
    get_trajectory, get_all_trajectories, ALL_TRAJECTORIES,
)


def parse_args():
    available = list(ALL_TRAJECTORIES.keys()) + ["all"]
    parser = argparse.ArgumentParser(
        description="Test 4: Trajectory Scenario — PD controller with Virtual Camera + optional noise"
    )
    # Scenario selection
    parser.add_argument("--scenario", type=str, default="straight_walk",
                        choices=available,
                        help="Trajectory scenario to run (default: straight_walk)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Target walking speed in m/s (default: 1.0)")

    # Noise control
    parser.add_argument("--noise", action="store_true", default=False,
                        help="Enable vision imperfections (noise, latency, dropouts)")
    parser.add_argument("--compare", action="store_true", default=False,
                        help="Run each scenario twice (ideal + noisy) for comparison")

    # Noise parameters (only used when --noise or --compare)
    parser.add_argument("--noise-sigma-xy", type=float, default=0.015,
                        help="Gaussian noise std on e_x, e_y (default: 0.015)")
    parser.add_argument("--noise-sigma-area", type=float, default=0.03,
                        help="Multiplicative Gaussian noise std on area (default: 0.03)")
    parser.add_argument("--latency-frames", type=int, default=1,
                        help="Detection latency in frames (default: 1)")
    parser.add_argument("--dropout-prob", type=float, default=0.02,
                        help="Per-frame detection dropout probability (default: 0.02)")
    parser.add_argument("--id-switch-prob", type=float, default=0.005,
                        help="Per-frame tracker ID switch probability (default: 0.005)")

    # Timing
    parser.add_argument("--settle-before", type=float, default=5.0,
                        help="Seconds to settle before trajectory starts (default: 5.0)")
    parser.add_argument("--takeoff-alt", type=float, default=10.0,
                        help="Takeoff altitude in meters (default: 10.0)")
    parser.add_argument("--loop-rate", type=float, default=20.0,
                        help="Control loop rate in Hz (default: 20.0)")

    return parser.parse_args()


def run_scenario(args, config, flight, trajectory, noise_mode, run_name):
    """
    Run a single trajectory scenario with the PD controller.

    Args:
        args: Parsed CLI arguments
        config: AppConfig
        flight: Connected FlightController
        trajectory: Trajectory object
        noise_mode: "ideal" or "noisy"
        run_name: Subdirectory name for logs

    Returns:
        Path to the output CSV file
    """
    print(f"\n{'═' * 60}")
    print(f"  SCENARIO: {trajectory.name} | MODE: {noise_mode}")
    print(f"  {trajectory.description}")
    print(f"  Duration: {args.settle_before}s settle + {trajectory.duration:.1f}s trajectory")
    print(f"{'═' * 60}")

    # Wait for position data
    if not wait_for_position_data(flight):
        print("ERROR: No position data. Skipping scenario.")
        return None

    # Get initial drone position and heading
    flight.poll_heartbeat()
    pos = flight.poll_local_position_ned()
    attitude = flight.poll_attitude()

    if pos is None:
        print("ERROR: Cannot read initial position. Skipping scenario.")
        return None

    initial_yaw = attitude.yaw  # Frozen heading for trajectory coordinate transform
    initial_distance = config.calibration.desired_stopping_distance_m

    # Place target at desired_stopping_distance in front of drone (equilibrium start)
    start_target_x = pos.x + initial_distance * math.cos(initial_yaw)
    start_target_y = pos.y + initial_distance * math.sin(initial_yaw)
    start_target_z = pos.z  # Same altitude as drone

    print(f"  Drone at:  ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), yaw={math.degrees(initial_yaw):.1f}°")
    print(f"  Target at: ({start_target_x:.2f}, {start_target_y:.2f}, {start_target_z:.2f})")
    print(f"  Initial distance: {initial_distance:.1f}m (desired stopping distance)")

    # Create virtual camera with calibrated optical constant
    from classes.distance_estimator import DistanceEstimator
    dist_estimator = DistanceEstimator(config.calibration)

    vcam = VirtualCamera(
        hfov_deg=62.2,
        vfov_deg=48.8,
        optical_constant=dist_estimator._optical_constant,
    )

    # Create camera with or without noise
    if noise_mode == "noisy":
        camera = NoisyVirtualCamera(
            vcam=vcam,
            noise_sigma_xy=args.noise_sigma_xy,
            noise_sigma_area=args.noise_sigma_area,
            latency_frames=args.latency_frames,
            dropout_prob=args.dropout_prob,
            id_switch_prob=args.id_switch_prob,
            seed=42,
        )
        print(f"  Noise: σ_xy={args.noise_sigma_xy}, σ_area={args.noise_sigma_area}, "
              f"latency={args.latency_frames}f, dropout={args.dropout_prob:.1%}, "
              f"id_switch={args.id_switch_prob:.1%}")
    else:
        camera = create_ideal_camera(vcam)
        print("  Noise: DISABLED (ideal mode)")

    # CSV logger
    csv_filename = os.path.join(run_name, f"test_4_{trajectory.name}_{noise_mode}.csv")
    logger = CSVLogger(csv_filename, [
        "time_s", "phase", "scenario",
        "target_x", "target_y", "target_z",
        "drone_x", "drone_y", "drone_z", "drone_yaw_deg",
        "e_x", "e_y", "e_dist_m", "e_mag",
        "e_x_clean", "e_y_clean", "e_dist_m_clean",
        "vx_cmd", "vz_cmd", "omega_z_cmd",
        "distance_to_target", "distance_clean",
        "detected", "id_switched", "noise_mode",
    ])

    # Load PD gains from config
    c = config.control
    k_p_yaw = c.k_p_yaw
    k_d_yaw = c.k_d_yaw
    k_p_vz = c.k_p_vz
    k_d_vz = c.k_d_vz
    k_p_vx = c.k_p_vx
    k_d_vx = c.k_d_vx

    # PD state variables
    prev_e_x = 0.0
    prev_e_y = 0.0
    prev_e_dist = 0.0
    prev_time = time.time()

    loop_period = 1.0 / args.loop_rate
    total_time = args.settle_before + trajectory.duration

    print(f"\n  Timeline:")
    print(f"    [0s – {args.settle_before}s]  Settle on initial target")
    print(f"    [{args.settle_before}s – {total_time:.1f}s]  Trajectory active")
    print(f"    Total: {total_time:.1f}s\n")

    t_start = time.time()

    try:
        while True:
            t_now = time.time()
            t_elapsed = t_now - t_start

            if t_elapsed >= total_time:
                break

            # Determine phase and trajectory time
            if t_elapsed < args.settle_before:
                phase = "SETTLE"
                traj_t = 0.0  # Target stays at start during settle
            else:
                phase = "TRACK"
                traj_t = t_elapsed - args.settle_before

            # Get target position from trajectory (body frame offsets)
            dx_fwd, dx_lat, dz = trajectory.position_fn(traj_t)

            # Transform body-frame offsets to NED using initial drone heading
            # Forward:  (cos(yaw), sin(yaw))
            # Right:    (-sin(yaw), cos(yaw))
            cur_target_x = start_target_x + dx_fwd * math.cos(initial_yaw) - dx_lat * math.sin(initial_yaw)
            cur_target_y = start_target_y + dx_fwd * math.sin(initial_yaw) + dx_lat * math.cos(initial_yaw)
            cur_target_z = start_target_z + dz

            # Poll drone state from SITL
            flight.poll_heartbeat()
            pos = flight.poll_local_position_ned()
            attitude = flight.poll_attitude()

            if pos is None:
                time.sleep(loop_period)
                continue

            current_yaw = attitude.yaw

            # Project target through (noisy) virtual camera
            cam_out = camera.project(
                pos.x, pos.y, pos.z, current_yaw,
                cur_target_x, cur_target_y, cur_target_z,
            )

            # ── Handle detection dropout ──────────────────────────────────
            # Matches mission_controller.py: send stop when target not found
            if not cam_out.detected:
                flight.send_velocity(0.0, 0.0, 0.0, 0.0)
                # Don't update PD memory during dropout
                logger.log(
                    f"{t_elapsed:.4f}", phase, trajectory.name,
                    f"{cur_target_x:.4f}", f"{cur_target_y:.4f}", f"{cur_target_z:.4f}",
                    f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}", f"{math.degrees(current_yaw):.2f}",
                    "0.0", "0.0", "0.0", "0.0",
                    f"{cam_out.clean_e_x:.6f}", f"{cam_out.clean_e_y:.6f}", "0.0",
                    "0.0", "0.0", "0.0",
                    f"{cam_out.distance:.4f}", f"{cam_out.clean_distance:.4f}",
                    "False", "False", noise_mode,
                )
                t_sleep = loop_period - (time.time() - t_now)
                if t_sleep > 0:
                    time.sleep(t_sleep)
                continue

            # ── Handle tracker ID switch ──────────────────────────────────
            # Matches mission_controller._reset_tracking_memory(): zeros prev errors
            if cam_out.id_switched:
                prev_e_x = 0.0
                prev_e_y = 0.0
                prev_e_dist = 0.0
                prev_time = t_now

            # ── Compute errors ────────────────────────────────────────────
            e_x = cam_out.e_x
            e_y = cam_out.e_y

            # Distance from noisy area (matches real system using bbox area)
            estimated_distance = dist_estimator.distance_to(cam_out.fake_area)
            e_dist_m = estimated_distance - config.calibration.desired_stopping_distance_m
            e_mag = min(1.0, math.sqrt(e_x ** 2 + e_y ** 2))

            # Clean values for logging comparison
            clean_distance = dist_estimator.distance_to(cam_out.clean_fake_area)
            e_dist_m_clean = clean_distance - config.calibration.desired_stopping_distance_m

            # ── PD derivative calculation ─────────────────────────────────
            dt = t_now - prev_time
            if 0 < dt < config.control.max_derivative_dt and not cam_out.id_switched:
                d_x = (e_x - prev_e_x) / dt
                d_y = (e_y - prev_e_y) / dt
                d_dist = (e_dist_m - prev_e_dist) / dt
            else:
                d_x = d_y = d_dist = 0.0

            # ── PD control output ─────────────────────────────────────────
            # Identical to mission_controller.py _process_frame() logic

            # Yaw (turn): centers the target horizontally
            omega_z = k_p_yaw * e_x + k_d_yaw * d_x

            # Vertical velocity: centers the target vertically
            v_z = k_p_vz * e_y + k_d_vz * d_y

            # Forward velocity: maintains desired distance
            v_x_request = k_p_vx * e_dist_m + k_d_vx * d_dist

            # ── Deadzones ─────────────────────────────────────────────────
            if abs(omega_z) < c.yaw_deadzone:
                omega_z = 0.0
            if abs(v_z) < c.vz_deadzone:
                v_z = 0.0
            if abs(e_dist_m) < c.dist_deadzone:
                v_x_request = 0.0

            # ── Coupled forward velocity limiting ─────────────────────────
            # Safety: don't fly forward if target is far off-center
            e_scaled = min(1.0, e_mag / c.r_stop)
            if e_scaled >= 1.0:
                v_x_limit = 0.0
            else:
                v_x_limit = c.max_vx * (1 - e_scaled ** 2)

            if v_x_request > 0:
                v_x = min(v_x_request, v_x_limit)
            else:
                v_x = max(v_x_request, -v_x_limit)

            # ── Standard clipping ─────────────────────────────────────────
            v_z = max(min(v_z, c.max_vz), -c.max_vz)
            omega_z = max(min(omega_z, c.max_yaw_rate), -c.max_yaw_rate)

            # ── Send velocity command ─────────────────────────────────────
            flight.send_velocity(v_x, 0.0, v_z, omega_z)

            # ── Log ───────────────────────────────────────────────────────
            logger.log(
                f"{t_elapsed:.4f}", phase, trajectory.name,
                f"{cur_target_x:.4f}", f"{cur_target_y:.4f}", f"{cur_target_z:.4f}",
                f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}", f"{math.degrees(current_yaw):.2f}",
                f"{e_x:.6f}", f"{e_y:.6f}", f"{e_dist_m:.6f}", f"{e_mag:.6f}",
                f"{cam_out.clean_e_x:.6f}", f"{cam_out.clean_e_y:.6f}", f"{e_dist_m_clean:.6f}",
                f"{v_x:.4f}", f"{v_z:.4f}", f"{omega_z:.4f}",
                f"{estimated_distance:.4f}", f"{clean_distance:.4f}",
                f"{cam_out.detected}", f"{cam_out.id_switched}", noise_mode,
            )

            # ── Update PD memory ──────────────────────────────────────────
            prev_e_x = e_x
            prev_e_y = e_y
            prev_e_dist = e_dist_m
            prev_time = t_now

            # ── Print progress every 1s ───────────────────────────────────
            if int(t_elapsed) != int(t_elapsed - loop_period):
                det_flag = "" if cam_out.detected else " [DROPOUT]"
                ids_flag = " [ID SWITCH]" if cam_out.id_switched else ""
                print(f"  t={t_elapsed:6.1f}s [{phase:6s}] | "
                      f"e_x={e_x:7.4f} e_y={e_y:7.4f} dist={estimated_distance:5.2f}m | "
                      f"vx={v_x:5.2f} vz={v_z:5.2f} ωz={omega_z:5.2f}"
                      f"{det_flag}{ids_flag}")

            # ── Sleep to maintain loop rate ───────────────────────────────
            t_sleep = loop_period - (time.time() - t_now)
            if t_sleep > 0:
                time.sleep(t_sleep)

    except KeyboardInterrupt:
        print("\n\n  Interrupted by user.")
    finally:
        flight.send_stop()
        logger.close()

    # Print noise statistics
    if noise_mode == "noisy":
        print(f"\n  {camera.stats_summary()}")

    print(f"\n  Scenario '{trajectory.name}' ({noise_mode}) complete → {logger.filepath}")
    return logger.filepath


def stabilize(flight, duration: float = 5.0):
    """Send stop commands for a few seconds to let the drone stabilize."""
    print(f"  Stabilizing for {duration:.0f}s...")
    steps = int(duration * 10)
    for _ in range(steps):
        flight.send_velocity(0.0, 0.0, 0.0, 0.0)
        flight.poll_heartbeat()
        time.sleep(0.1)


def main():
    args = parse_args()

    print("=" * 60)
    print("  TEST 4: TRAJECTORY SCENARIO SIMULATION")
    print(f"  Scenario: {args.scenario} | Speed: {args.speed} m/s")
    if args.compare:
        print("  Mode: COMPARE (ideal + noisy)")
    elif args.noise:
        print("  Mode: NOISY")
    else:
        print("  Mode: IDEAL (no noise)")
    print("=" * 60)

    # Load config and connect
    config = load_config()
    flight = sitl_connect(config)

    # Arm and take off
    if not sitl_arm_and_takeoff(flight, target_alt=args.takeoff_alt):
        print("ERROR: Takeoff failed. Exiting.")
        return

    # Determine scenarios
    if args.scenario == "all":
        scenarios = get_all_trajectories(speed=args.speed)
    else:
        scenarios = [get_trajectory(args.scenario, speed=args.speed)]

    # Determine noise modes
    if args.compare:
        noise_modes = ["ideal", "noisy"]
        mode_str = "compare"
    elif args.noise:
        noise_modes = ["noisy"]
        mode_str = "noisy"
    else:
        noise_modes = ["ideal"]
        mode_str = "ideal"

    # Auto-generate run name
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    existing_runs = glob.glob(os.path.join(logs_dir, "run_*"))
    run_numbers = []
    for r in existing_runs:
        dirname = os.path.basename(r)
        try:
            num = int(dirname.split("_")[1])
            run_numbers.append(num)
        except (IndexError, ValueError):
            pass
    next_run = max(run_numbers) + 1 if run_numbers else 1
    run_name = f"run_{next_run:03d}_{args.scenario}_{mode_str}"
    run_dir = os.path.join(logs_dir, run_name)
    os.makedirs(run_dir, exist_ok=True)

    # Save run metadata
    with open(os.path.join(run_dir, "metadata.txt"), "w") as f:
        f.write("TEST 4: TRAJECTORY SCENARIO\n")
        f.write("=" * 30 + "\n")
        f.write(f"Scenarios: {[s.name for s in scenarios]}\n")
        f.write(f"Noise Modes: {noise_modes}\n")
        f.write(f"Walking Speed: {args.speed} m/s\n")
        f.write(f"Settle Time: {args.settle_before} s\n")
        f.write(f"Loop Rate: {args.loop_rate} Hz\n")
        f.write(f"\nNOISE PARAMETERS\n")
        f.write("=" * 30 + "\n")
        f.write(f"sigma_xy: {args.noise_sigma_xy}\n")
        f.write(f"sigma_area: {args.noise_sigma_area}\n")
        f.write(f"latency_frames: {args.latency_frames}\n")
        f.write(f"dropout_prob: {args.dropout_prob}\n")
        f.write(f"id_switch_prob: {args.id_switch_prob}\n")
        f.write(f"\nCONTROL GAINS (from config)\n")
        f.write("=" * 30 + "\n")
        f.write(f"k_p_yaw: {config.control.k_p_yaw}\n")
        f.write(f"k_d_yaw: {config.control.k_d_yaw}\n")
        f.write(f"k_p_vz: {config.control.k_p_vz}\n")
        f.write(f"k_d_vz: {config.control.k_d_vz}\n")
        f.write(f"k_p_vx: {config.control.k_p_vx}\n")
        f.write(f"k_d_vx: {config.control.k_d_vx}\n")
        f.write(f"desired_stopping_distance: {config.calibration.desired_stopping_distance_m}\n")

    print(f"\n  Run directory: {run_dir}\n")

    # Run scenarios
    output_files = []
    try:
        for i, traj in enumerate(scenarios):
            for j, mode in enumerate(noise_modes):
                filepath = run_scenario(args, config, flight, traj, mode, run_name)
                if filepath:
                    output_files.append(filepath)

                # Stabilize between runs (but not after the very last one)
                is_last = (i == len(scenarios) - 1) and (j == len(noise_modes) - 1)
                if not is_last:
                    stabilize(flight)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    finally:
        flight.send_stop()

    # Summary
    print(f"\n{'=' * 60}")
    print(f"  TEST 4 COMPLETE")
    print(f"  Output files:")
    for f in output_files:
        print(f"    • {f}")
    print(f"\n  To generate plots:")
    print(f"    python sitl_tests/test_4_trajectory/plot_test_4.py")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
