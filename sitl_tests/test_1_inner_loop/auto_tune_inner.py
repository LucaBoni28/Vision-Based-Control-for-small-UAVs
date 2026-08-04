###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Automated Inner Loop Tuner
#         Iterates through a range of P and I gains for the horizontal
#         velocity controller (PSC_NE_VEL_P, PSC_NE_VEL_I).
#         For each combination, it sets the parameters via MAVLink, runs
#         a short step response, and computes a performance score.
#
# Usage:  python sitl_tests/auto_tune_inner.py --p-min 1.0 --p-max 3.0 --p-step 0.5
###############################################################################

import argparse
import time
import sys
import os
import glob
import pandas as pd
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))

from sitl_tests.utils.sitl_utils import (
    load_config, sitl_connect, sitl_arm_and_takeoff,
    wait_for_position_data, CSVLogger,
)
from plot_test_1 import compute_metrics


def parse_args():
    parser = argparse.ArgumentParser(
        description="Automated Inner Loop Tuner (Velocity XY)"
    )
    # Gain ranges
    parser.add_argument("--p-min", type=float, default=2.0, help="Min P gain (default: 2.0)")
    parser.add_argument("--p-max", type=float, default=4.0, help="Max P gain (default: 4.0)")
    parser.add_argument("--p-step", type=float, default=1.0, help="Step P gain (default: 1.0)")
    
    parser.add_argument("--i-min", type=float, default=1.0, help="Min I gain (default: 1.0)")
    parser.add_argument("--i-max", type=float, default=2.0, help="Max I gain (default: 2.0)")
    parser.add_argument("--i-step", type=float, default=1.0, help="Step I gain (default: 1.0)")
    
    # Test params
    parser.add_argument("--axis", type=str, default="vx", choices=["vx", "vz", "yaw_rate"], 
                        help="Axis to tune (default: vx)")
    parser.add_argument("--velocity", type=float, default=1.0, help="Step velocity (default: 1.0 m/s)")
    parser.add_argument("--duration", type=float, default=8.0, help="Duration of step (default: 8s)")
    parser.add_argument("--settle-time", type=float, default=3.0, help="Hover time before step (default: 3s)")
    parser.add_argument("--takeoff-alt", type=float, default=10.0, help="Takeoff altitude (default: 10m)")
    return parser.parse_args()

def run_single_test(flight, p_val, i_val, args, test_id):
    """Run a step response for a specific P and I combination."""
    
    # Determine parameters based on axis
    if args.axis == "vx":
        param_p = "PSC_NE_VEL_P"
        param_i = "PSC_NE_VEL_I"
        cmd_col = "cmd_vx"
        act_col = "actual_vx"
    elif args.axis == "vz":
        param_p = "PSC_D_VEL_P"
        param_i = "PSC_D_VEL_I"
        cmd_col = "cmd_vz"
        act_col = "actual_vz"
    elif args.axis == "yaw_rate":
        param_p = "ATC_RAT_YAW_P"
        param_i = "ATC_RAT_YAW_I"
        cmd_col = "cmd_yaw_rate"
        act_col = "actual_yaw_rate"

    # 1. Set Parameters via MAVLink
    print(f"\n--- Testing {args.axis.upper()} | {param_p}={p_val:.2f}, {param_i}={i_val:.2f} ---")
    flight.set_parameter(param_p, p_val)
    flight.set_parameter(param_i, i_val)
    time.sleep(1.0) # Give SITL a moment to apply parameters
    
    # Ensure drone is stopped
    flight.send_stop()
    time.sleep(2.0)
    
    # 2. Setup Logger
    csv_filename = f"autotune_{args.axis}_p{p_val:.2f}_i{i_val:.2f}_{test_id}.csv"
    logger = CSVLogger(csv_filename, [
        "time_s", cmd_col, act_col
    ])
    
    total_baseline = args.settle_time
    total_step = args.duration
    total_time = total_baseline + total_step
    
    t_start = time.time()
    step_start_time = total_baseline
    
    loop_period = 1.0 / 20.0
    
    try:
        while True:
            t_now = time.time()
            t_elapsed = t_now - t_start
            
            if t_elapsed >= total_time:
                break
                
            # Determine command
            if t_elapsed < total_baseline:
                cmd = 0.0
            else:
                cmd = args.velocity
                
            # Send command
            if args.axis == "vx":
                flight.send_velocity(cmd, 0.0, 0.0, 0.0)
            elif args.axis == "vz":
                flight.send_velocity(0.0, 0.0, cmd, 0.0)
            elif args.axis == "yaw_rate":
                flight.send_velocity(0.0, 0.0, 0.0, cmd)
            
            # Poll state
            flight.poll_heartbeat()
            pos = flight.poll_local_position_ned()
            att = flight.poll_attitude()
            
            if pos is not None and att is not None:
                if args.axis == "vx":
                    act = pos.vx
                elif args.axis == "vz":
                    act = pos.vz
                elif args.axis == "yaw_rate":
                    act = att.yawspeed
                    
                logger.log(f"{t_elapsed:.4f}", f"{cmd:.4f}", f"{act:.4f}")
                
            # Sleep
            t_sleep = loop_period - (time.time() - t_now)
            if t_sleep > 0:
                time.sleep(t_sleep)
                
    except KeyboardInterrupt:
        print("Test interrupted.")
        raise
    finally:
        flight.send_stop()
        logger.close()
        
    return logger.filepath, step_start_time


def main():
    args = parse_args()
    
    if args.axis == "vz" and args.velocity > 0:
        print("Note: Inverting vz velocity to climb instead of descend to avoid hitting the ground.")
        args.velocity = -args.velocity
    
    # Generate parameter grid
    # Use np.arange but handle floating point issues by adding a small epsilon to max
    p_values = np.arange(args.p_min, args.p_max + 1e-5, args.p_step)
    i_values = np.arange(args.i_min, args.i_max + 1e-5, args.i_step)
    
    total_tests = len(p_values) * len(i_values)
    estimated_time = total_tests * (args.duration + args.settle_time + 4.0)
    
    print("=" * 60)
    print("  AUTOMATED INNER LOOP TUNER")
    print(f"  P range: {args.p_min} to {args.p_max} (step {args.p_step}) -> {p_values}")
    print(f"  I range: {args.i_min} to {args.i_max} (step {args.i_step}) -> {i_values}")
    print(f"  Total tests: {total_tests}")
    print(f"  Estimated time: {estimated_time / 60:.1f} minutes")
    print("=" * 60)
    
    # Load config and connect
    config = load_config()
    flight = sitl_connect(config)

    # Arm and take off
    if not sitl_arm_and_takeoff(flight, target_alt=args.takeoff_alt):
        print("ERROR: Takeoff failed. Exiting.")
        return
        
    if not wait_for_position_data(flight):
        print("ERROR: No position data. Exiting.")
        return
        
    # Unique run ID based on existing files
    logs_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    existing_summaries = glob.glob(os.path.join(logs_dir_path, f"autotune_{args.axis}_summary_run_*.csv"))
    run_numbers = []
    for f in existing_summaries:
        try:
            num = int(f.split("_run_")[-1].split(".")[0])
            run_numbers.append(num)
        except ValueError:
            pass
    next_run = max(run_numbers) + 1 if run_numbers else 1
    test_id = f"run_{next_run:03d}"
    
    results = []
    best_cost = float('inf')
    best_file = None
    
    try:
        test_num = 1
        for p in p_values:
            for i in i_values:
                print(f"\n[Test {test_num}/{total_tests}]")
                
                # Regain altitude if we lost too much during previous VZ tests
                if args.axis == "vz":
                    alt = flight.poll_relative_alt() or 0.0
                    if alt < (args.takeoff_alt - 2.0):
                        print(f"  Altitude low ({alt:.1f}m). Climbing back to {args.takeoff_alt}m before next test...")
                        flight.send_velocity(0.0, 0.0, -1.5, 0.0)  # negative vz = climb
                        while alt < args.takeoff_alt:
                            time.sleep(0.5)
                            alt = flight.poll_relative_alt() or 0.0
                        flight.send_stop()
                        time.sleep(2.0)
                
                csv_path, step_start = run_single_test(flight, p, i, args, test_id)
                
                # Compute metrics
                df = pd.read_csv(csv_path)
                
                if args.axis == "vx":
                    metrics = compute_metrics(df, "cmd_vx", "actual_vx", step_start)
                elif args.axis == "vz":
                    metrics = compute_metrics(df, "cmd_vz", "actual_vz", step_start)
                elif args.axis == "yaw_rate":
                    metrics = compute_metrics(df, "cmd_yaw_rate", "actual_yaw_rate", step_start)
                
                if metrics:
                    # Calculate a 'Cost' score (lower is better)
                    # Example cost: Rise Time (s) + 2 * Steady State Error
                    rt = metrics.get('rise_time_s', float('nan'))
                    sse = metrics.get('steady_state_error', float('nan'))
                    
                    if np.isnan(rt) or np.isnan(sse):
                        cost = float('inf')
                    else:
                        cost = rt + (2.0 * sse)
                        
                    results.append({
                        "P": p, "I": i,
                        "Rise_Time": rt,
                        "SS_Error": sse,
                        "Cost": cost,
                        "File": csv_path
                    })
                    
                    print(f"  Result -> Rise Time: {rt:.2f}s, SS Error: {sse:.3f} m/s | COST: {cost:.3f}")
                    
                    # Keep only the best CSV file on disk to save space
                    if cost < best_cost:
                        if best_file is not None and os.path.exists(best_file):
                            os.remove(best_file)
                        best_cost = cost
                        best_file = csv_path
                    else:
                        # Not the best, delete it immediately
                        if os.path.exists(csv_path):
                            os.remove(csv_path)
                else:
                    print("  Result -> Failed to compute metrics.")
                    if os.path.exists(csv_path):
                        os.remove(csv_path)
                    
                test_num += 1
                
    except KeyboardInterrupt:
        print("\n\nTuning interrupted!")
    finally:
        flight.send_stop()
        
    print(f"\n{'=' * 60}")
    print("  TUNING COMPLETE")
    print(f"{'=' * 60}")
    
    if results:
        results_df = pd.DataFrame(results)
        print(results_df.sort_values(by="Cost")[["P", "I", "Rise_Time", "SS_Error", "Cost"]].to_string(index=False))
        
        best = results_df.loc[results_df["Cost"].idxmin()]
        print(f"\n* BEST TUNE ({args.axis.upper()}):")
        print(f"   P-Gain = {best['P']}")
        print(f"   I-Gain = {best['I']}")
        print(f"   (Score: {best['Cost']:.3f})")
        print(f"   Plot file: {best['File']} using plot_test_1.py to see it!")
        
        # Save results to CSV
        logs_dir = os.path.dirname(best['File'])
        summary_file = os.path.join(logs_dir, f"autotune_{args.axis}_summary_{test_id}.csv")
        results_df.to_csv(summary_file, index=False)
        print(f"\nSaved summary to: {summary_file}")
    else:
        print("No results collected.")

if __name__ == "__main__":
    main()
