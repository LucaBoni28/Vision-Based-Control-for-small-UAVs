###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Test 1 — Inner Loop Verification (Step Input)
#
#         Bypasses the vision PID entirely. Sends a direct velocity step command
#         to ArduPilot SITL and logs the actual vs. commanded velocity to verify
#         that the inner flight controller tracks velocity commands accurately.
#
#         Output: graphs_generation/logs/test_1_step_<axis>.csv
#
# Usage:  python sitl_tests/test_1_inner_loop.py
#         python sitl_tests/test_1_inner_loop.py --velocity 1.5 --axis vx --duration 15
###############################################################################

import argparse
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from sitl_tests.utils.sitl_utils import (
    load_config, sitl_connect, sitl_arm_and_takeoff,
    wait_for_position_data, CSVLogger,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test 1: Inner Loop Verification — Step velocity input to ArduPilot SITL"
    )
    parser.add_argument("--velocity", type=float, default=1.0,
                        help="Step velocity magnitude in m/s (default: 1.0)")
    parser.add_argument("--axis", type=str, default="vx",
                        choices=["vx", "vy", "vz"],
                        help="Axis to apply the step input (default: vx)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Duration of the step in seconds (default: 10.0)")
    parser.add_argument("--settle-time", type=float, default=5.0,
                        help="Time to record BEFORE the step (baseline) in seconds (default: 5.0)")
    parser.add_argument("--takeoff-alt", type=float, default=10.0,
                        help="Takeoff altitude in meters (default: 10.0)")
    parser.add_argument("--loop-rate", type=float, default=20.0,
                        help="Control loop rate in Hz (default: 20.0)")
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("  TEST 1: INNER LOOP VERIFICATION (STEP INPUT)")
    print(f"  Axis: {args.axis} | Velocity: {args.velocity} m/s | Duration: {args.duration}s")
    print("=" * 60)

    # Load config and connect to SITL
    config = load_config()
    flight = sitl_connect(config)

    # Arm and take off
    if not sitl_arm_and_takeoff(flight, target_alt=args.takeoff_alt):
        print("ERROR: Takeoff failed. Exiting.")
        return

    # Wait for position data
    if not wait_for_position_data(flight):
        print("ERROR: No position data. Exiting.")
        return

    # Determine run number
    import glob
    logs_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    existing_logs = glob.glob(os.path.join(logs_dir_path, f"test_1_step_{args.axis}_run_*.csv"))
    run_numbers = []
    for f in existing_logs:
        try:
            num = int(f.split("_run_")[-1].split(".")[0])
            run_numbers.append(num)
        except ValueError:
            pass
    next_run = max(run_numbers) + 1 if run_numbers else 1
    run_id = f"run_{next_run:03d}"
    
    # Prepare CSV logger
    csv_filename = f"test_1_step_{args.axis}_{run_id}.csv"
    logger = CSVLogger(csv_filename, [
        "time_s",
        "cmd_vx", "cmd_vy", "cmd_vz",
        "actual_vx", "actual_vy", "actual_vz", "actual_yaw_rate",
        "pos_x", "pos_y", "pos_z",
    ])

    loop_period = 1.0 / args.loop_rate

    # Build the step command vector
    def get_cmd(step_active: bool):
        """Returns (vx, vy, vz) based on the active axis."""
        vx, vy, vz = 0.0, 0.0, 0.0
        if step_active:
            if args.axis == "vx":
                vx = args.velocity
            elif args.axis == "vy":
                vy = args.velocity
            elif args.axis == "vz":
                vz = args.velocity
        return vx, vy, vz

    # ── Timing ──
    total_baseline = args.settle_time
    total_step = args.duration
    total_after = args.settle_time  # Record some time after the step too

    total_time = total_baseline + total_step + total_after
    print(f"\nRecording timeline:")
    print(f"  [0s – {total_baseline}s]   Baseline (hover)")
    print(f"  [{total_baseline}s – {total_baseline + total_step}s]  Step input ({args.axis} = {args.velocity} m/s)")
    print(f"  [{total_baseline + total_step}s – {total_time}s]  Recovery (hover)")
    print(f"  Total: {total_time}s at {args.loop_rate} Hz\n")

    t_start = time.time()

    try:
        while True:
            t_now = time.time()
            t_elapsed = t_now - t_start

            if t_elapsed >= total_time:
                break

            # Determine phase
            step_active = total_baseline <= t_elapsed < total_baseline + total_step

            # Get command and send to ArduPilot
            cmd_vx, cmd_vy, cmd_vz = get_cmd(step_active)
            flight.send_velocity(cmd_vx, cmd_vy, cmd_vz, 0.0)

            # Poll actual state
            flight.poll_heartbeat()
            pos = flight.poll_local_position_ned()
            att = flight.poll_attitude()

            if pos is not None and att is not None:
                logger.log(
                    f"{t_elapsed:.4f}",
                    f"{cmd_vx:.4f}", f"{cmd_vy:.4f}", f"{cmd_vz:.4f}",
                    f"{pos.vx:.4f}", f"{pos.vy:.4f}", f"{pos.vz:.4f}",
                    f"{att.yawspeed:.4f}",
                    f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}",
                )

            # Print progress every 0.5s
            if int(t_elapsed * 2) != int((t_elapsed - loop_period) * 2):
                phase = "STEP" if step_active else "HOVER"
                cmd_val = args.velocity if step_active else 0.0
                if pos:
                    print(f"  t={t_elapsed:6.2f}s [{phase:5s}] | cmd={args.axis}={cmd_val:5.2f} | "
                          f"actual_vx={pos.vx:6.3f} vy={pos.vy:6.3f} vz={pos.vz:6.3f}")

            # Sleep to maintain loop rate
            t_sleep = loop_period - (time.time() - t_now)
            if t_sleep > 0:
                time.sleep(t_sleep)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        print("\nSending stop command...")
        flight.send_stop()
        logger.close()

    print(f"\n{'=' * 60}")
    print(f"  TEST 1 COMPLETE")
    print(f"  Data saved to: {logger.filepath}")
    print(f"  Analyze with Mission Planner .tlog for cross-validation")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
