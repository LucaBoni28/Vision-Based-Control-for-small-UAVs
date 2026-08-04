###############################################################################
# Author: Luca Boninsegna (modified for outer loop)
# Date:   29/07/2026
# Descr:  Automated Outer Loop Tuner
#         Iterates through a range of P and D gains for the vision PID controller.
#         For each combination, it modifies the config, runs
#         a short step response, and computes a performance score.
#
# Usage:  python sitl_tests/test_2_step_response/auto_tune_outer.py --axis dist
###############################################################################

import argparse
import time
import sys
import os
import pandas as pd
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from sitl_tests.utils.sitl_utils import (
    load_config, sitl_connect, sitl_arm_and_takeoff, wait_for_position_data
)
from sitl_tests.test_2_step_response.plot_test_2 import compute_step_metrics
from sitl_tests.test_2_step_response.test_2_step_response import run_step_response


def parse_args():
    parser = argparse.ArgumentParser(
        description="Automated Outer Loop Tuner (Vision PID)"
    )
    # Gain ranges
    parser.add_argument("--p-min", type=float, default=0.5, help="Min P gain (default: 0.5)")
    parser.add_argument("--p-max", type=float, default=1.5, help="Max P gain (default: 1.5)")
    parser.add_argument("--p-step", type=float, default=0.5, help="Step P gain (default: 0.5)")
    
    parser.add_argument("--d-min", type=float, default=0.02, help="Min D gain (default: 0.02)")
    parser.add_argument("--d-max", type=float, default=0.10, help="Max D gain (default: 0.10)")
    parser.add_argument("--d-step", type=float, default=0.04, help="Step D gain (default: 0.04)")
    
    # Test params
    parser.add_argument("--axis", type=str, default="dist", choices=["dist", "yaw", "alt"], 
                        help="Axis to tune (default: dist)")
    parser.add_argument("--step-magnitude", type=float, default=None, 
                        help="Step magnitude (default: 3.0)")
    parser.add_argument("--initial-distance", type=float, default=None, 
                        help="Initial distance (default: 0.6 for dist, 5.0 for others)")
    parser.add_argument("--settle-before", type=float, default=5.0, 
                        help="Time to let PD settle before step (s)")
    parser.add_argument("--record-after", type=float, default=10.0, 
                        help="Time to record after the step (s)")
    parser.add_argument("--takeoff-alt", type=float, default=10.0, 
                        help="Takeoff altitude (default: 10m)")
    parser.add_argument("--loop-rate", type=float, default=20.0, 
                        help="Loop rate in Hz")
    
    # Needed for run_step_response to not fail
    parser.add_argument("--step-axis", type=str, default="")
    return parser.parse_args()


def main():
    args = parse_args()
    
    # Map 'axis' to 'step_axis' for compatibility with run_step_response
    args.step_axis = args.axis
    
    if args.step_magnitude is None:
        args.step_magnitude = 3.0
        
    if args.initial_distance is None:
        if args.step_axis == "dist":
            args.initial_distance = 0.6
        else:
            args.initial_distance = 5.0

    # Generate parameter grid
    p_values = np.arange(args.p_min, args.p_max + 1e-5, args.p_step)
    d_values = np.arange(args.d_min, args.d_max + 1e-5, args.d_step)
    
    total_tests = len(p_values) * len(d_values)
    estimated_time = total_tests * (args.settle_before + args.record_after + 4.0)
    
    print("=" * 60)
    print("  AUTOMATED OUTER LOOP TUNER")
    print(f"  Axis: {args.step_axis}")
    print(f"  P range: {args.p_min} to {args.p_max} (step {args.p_step}) -> {p_values}")
    print(f"  D range: {args.d_min} to {args.d_max} (step {args.d_step}) -> {d_values}")
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
    import glob
    logs_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    existing_summaries = glob.glob(os.path.join(logs_dir_path, f"autotune_outer_{args.axis}_summary_run_*.csv"))
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
            for d in d_values:
                print(f"\n[Test {test_num}/{total_tests}] Setting P={p:.2f}, D={d:.4f}")
                
                # Ensure the drone is perfectly stationary before starting the test
                # This prevents momentum from the previous test from skewing the settle phase
                flight.send_stop()
                time.sleep(3.0)
                
                # Regain altitude if we lost too much during alt tests
                if args.axis == "alt":
                    alt = flight.poll_relative_alt() or 0.0
                    if alt < (args.takeoff_alt - 2.0):
                        print(f"  Altitude low ({alt:.1f}m). Climbing back to {args.takeoff_alt}m...")
                        flight.send_velocity(0.0, 0.0, -1.5, 0.0)  # negative vz = climb
                        while alt < args.takeoff_alt:
                            time.sleep(0.5)
                            alt = flight.poll_relative_alt() or 0.0
                        flight.send_stop()
                        time.sleep(2.0)
                
                # Modify config in memory
                if args.axis == "dist":
                    config.control.k_p_vx = p
                    config.control.k_d_vx = d
                    err_col = "e_area"
                elif args.axis == "yaw":
                    config.control.k_p_yaw = p
                    config.control.k_d_yaw = d
                    err_col = "e_x"
                elif args.axis == "alt":
                    config.control.k_p_vz = p
                    config.control.k_d_vz = d
                    err_col = "e_y"
                
                csv_path, step_start = run_step_response(args, config, flight, test_id=f"p{p:.2f}_d{d:.4f}_{test_id}")
                
                if csv_path is None:
                    print("  ERROR running test. Skipping.")
                    continue
                
                # Compute metrics
                df = pd.read_csv(csv_path)
                time_s = df["time_s"].values
                signal = df[err_col].values
                
                metrics = compute_step_metrics(time_s, signal, step_start)
                
                if metrics and metrics.get('settling_time_s') is not None:
                    # Cost Function: Settling Time + 5 * Overshoot%
                    # We penalize overshoot heavily for a vision drone.
                    st = metrics['settling_time_s']
                    os_pct = metrics['overshoot_pct']
                    rt = metrics['rise_time_s']
                    ss_err = metrics.get('steady_state_error', 0.0)
                    
                    # Target loss condition (e_area near +/- 1.0 or +/- 2.0 when bounds are hit)
                    if ss_err > 0.5:
                        print("  Result -> FAILED (TARGET LOST OR HUGE STEADY STATE ERROR)")
                        os.remove(csv_path)
                        test_num += 1
                        continue

                    cost = st + (5.0 * (os_pct / 100.0)) + (100.0 * ss_err)
                        
                    results.append({
                        "P": p, "D": d,
                        "Rise_Time": rt,
                        "Overshoot": os_pct,
                        "Settling_Time": st,
                        "SS_Error": ss_err,
                        "Cost": cost,
                        "File": csv_path
                    })
                    
                    print(f"  Result -> Rise Time: {rt:.2f}s, Overshoot: {os_pct:.1f}%, Settling: {st:.2f}s, SS_Err: {ss_err:.3f} | COST: {cost:.3f}")
                    
                    # Keep only the best CSV file on disk to save space
                    if cost < best_cost:
                        if best_file is not None and os.path.exists(best_file):
                            os.remove(best_file)
                        best_cost = cost
                        best_file = csv_path
                    else:
                        os.remove(csv_path)
                else:
                    print("  Result -> FAILED TO SETTLE")
                    os.remove(csv_path)
                    
                test_num += 1
                
    except KeyboardInterrupt:
        print("\n\nAutotuning interrupted by user.")
        flight.send_stop()
    
    print("\n" + "=" * 60)
    print("  AUTOTUNING COMPLETE")
    
    if len(results) > 0:
        # Sort and print
        res_df = pd.DataFrame(results).sort_values("Cost")
        print("\nTOP 5 RESULTS:")
        print(res_df.head(5).to_string(index=False))
        
        best = res_df.iloc[0]
        print(f"\nBEST COMBINATION:")
        print(f"  P = {best['P']:.2f}, D = {best['D']:.4f} (Cost = {best['Cost']:.3f})")
        print(f"  Saved to: {best['File']}")
        
        # Save summary
        summary_path = os.path.join(os.path.dirname(best_file), f"autotune_outer_{args.axis}_summary_{test_id}.csv")
        res_df.to_csv(summary_path, index=False)
        print(f"  Summary saved to: {summary_path}")
    else:
        print("  No successful tuning results found.")
    
    print("=" * 60)


if __name__ == "__main__":
    main()
