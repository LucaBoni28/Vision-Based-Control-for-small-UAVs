###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Post-processing for Test 1 — Inner Loop Verification.
#         Reads the CSV produced by test_1_inner_loop.py and generates:
#           - Commanded vs. Actual velocity plot
#           - Tracking error over time
#           - Key metrics: delay, rise time, steady-state error
#
# Usage:  python graphs_generation/plot_test_1.py
#         python graphs_generation/plot_test_1.py --csv graphs_generation/logs/test_1_step_vx.csv
###############################################################################

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def find_latest_csv(axis=None):
    """Find the most recent Test 1 CSV file."""
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    if axis:
        pattern = os.path.join(logs_dir, f"test_1_step_{axis}.csv")
    else:
        pattern = os.path.join(logs_dir, "test_1_step_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def compute_metrics(df, cmd_col, actual_col, step_start_time):
    """
    Compute inner loop tracking metrics.

    Returns:
        dict with: delay_s, rise_time_s, steady_state_error, overshoot_pct
    """
    # Get data during the step phase by finding where the command is non-zero
    max_cmd = df[cmd_col].abs().max()
    if max_cmd < 0.001:
        return {}
        
    step_mask = df[cmd_col].abs() > (max_cmd * 0.5)
    step_data = df[step_mask].copy()

    if step_data.empty:
        return {}

    cmd_value = step_data[cmd_col].iloc[0]

    actual = step_data[actual_col].values
    times = step_data["time_s"].values

    # Steady-state: average of last 20% of step data
    n_ss = max(1, len(actual) // 5)
    steady_state = np.mean(actual[-n_ss:])
    steady_state_error = abs(cmd_value - steady_state)

    # Normalize response for rise time calculation
    # (assuming starting from ~0)
    baseline = np.mean(df[df["time_s"] < step_start_time][actual_col].values[-10:]) if len(df[df["time_s"] < step_start_time]) > 10 else 0.0
    response_range = cmd_value - baseline

    if abs(response_range) < 0.001:
        return {"steady_state_error": steady_state_error}

    normalized = (actual - baseline) / response_range

    # Delay time: time for response to first reach 10% of command
    delay_idx = np.where(normalized >= 0.1)[0]
    delay_s = times[delay_idx[0]] - step_start_time if len(delay_idx) > 0 else float('nan')

    # Rise time: 10% → 90%
    t10_idx = np.where(normalized >= 0.1)[0]
    t90_idx = np.where(normalized >= 0.9)[0]
    if len(t10_idx) > 0 and len(t90_idx) > 0:
        rise_time_s = times[t90_idx[0]] - times[t10_idx[0]]
    else:
        rise_time_s = float('nan')

    # Overshoot
    peak = np.max(actual) if cmd_value > 0 else np.min(actual)
    if abs(steady_state - baseline) > 0.001:
        overshoot_pct = abs((peak - steady_state) / (steady_state - baseline)) * 100
    else:
        overshoot_pct = 0.0

    return {
        "command": cmd_value,
        "steady_state": steady_state,
        "delay_s": delay_s,
        "rise_time_s": rise_time_s,
        "steady_state_error": steady_state_error,
        "overshoot_pct": overshoot_pct,
    }


def plot_test_1(csv_path, output_dir=None):
    """Generate Test 1 plots and compute metrics."""
    print(f"Reading: {csv_path}")
    df = pd.read_csv(csv_path)

    if output_dir is None:
        output_dir = os.path.dirname(csv_path)

    # Determine which axis was tested from the filename
    basename = os.path.basename(csv_path)
    if "yaw_rate" in basename:
        axis = "yaw_rate"
        cmd_col = "cmd_yaw_rate"
        actual_col = "actual_yaw_rate"
        ylabel = "Yaw Rate (rad/s)"
    elif "vy" in basename:
        axis = "vy"
        cmd_col = "cmd_vy"
        actual_col = "actual_vy"
        ylabel = "Velocity Y (m/s)"
    elif "vz" in basename:
        axis = "vz"
        cmd_col = "cmd_vz"
        actual_col = "actual_vz"
        ylabel = "Velocity Z (m/s)"
    else:
        axis = "vx"
        cmd_col = "cmd_vx"
        actual_col = "actual_vx"
        ylabel = "Velocity X (m/s)"

    # If the commanded step is negative (e.g. climbing for vz), 
    # invert the data so it plots and measures as a positive step for thesis readability
    max_cmd = df[cmd_col].max()
    min_cmd = df[cmd_col].min()
    if abs(min_cmd) > abs(max_cmd) and abs(min_cmd) > 0.1:
        print(f"Note: Inverting negative {axis} step data for plotting and metrics.")
        df[cmd_col] = -df[cmd_col]
        df[actual_col] = -df[actual_col]

    time_s = df["time_s"].values

    # Detect step start time (when command goes from 0 to non-zero)
    cmd = df[cmd_col].values
    step_indices = np.where(np.abs(np.diff(cmd)) > 0.01)[0]
    if len(step_indices) >= 1:
        step_start = time_s[step_indices[0] + 1]
        step_end = time_s[step_indices[-1] + 1] if len(step_indices) > 1 else time_s[-1]
    else:
        step_start = time_s[0]
        step_end = time_s[-1]

    # ── Figure 1: Commanded vs Actual Velocity ──────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1]})

    ax1 = axes[0]
    ax1.plot(time_s, df[cmd_col], 'r-', linewidth=2, label=f"Commanded {axis}", alpha=0.9)
    ax1.plot(time_s, df[actual_col], 'b-', linewidth=1.5, label=f"Actual {axis}", alpha=0.8)
    ax1.axvline(x=step_start, color='green', linestyle='--', alpha=0.5, label="Step start")
    if len(step_indices) > 1:
        ax1.axvline(x=step_end, color='orange', linestyle='--', alpha=0.5, label="Step end")
    ax1.set_ylabel(ylabel, fontsize=12)
    ax1.set_title(f"Test 1: Inner Loop Velocity Tracking — {axis.upper()}", fontsize=14, fontweight='bold')
    ax1.legend(loc="upper right", fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Tracking error
    ax2 = axes[1]
    tracking_error = df[cmd_col].values - df[actual_col].values
    ax2.plot(time_s, tracking_error, 'k-', linewidth=1, alpha=0.7)
    ax2.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    ax2.fill_between(time_s, tracking_error, alpha=0.2, color='red')
    ax2.set_xlabel("Time (s)", fontsize=12)
    ax2.set_ylabel("Error (m/s)", fontsize=12)
    ax2.set_title("Tracking Error", fontsize=11)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save to plots/ directory next to the logs/ directory
    if os.path.basename(output_dir) == "logs":
        plots_dir = os.path.join(os.path.dirname(output_dir), "plots")
    else:
        plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    
    # Use the CSV base name for the plot
    plot_basename = os.path.splitext(basename)[0]
    plot_path = os.path.join(plots_dir, f"{plot_basename}.pdf")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plot_path_png = plot_path.replace('.pdf', '.png')
    plt.savefig(plot_path_png, dpi=150, bbox_inches='tight')
    print(f"Saved: {plot_path}")
    print(f"Saved: {plot_path_png}")
    plt.close()

    # ── Compute Metrics ─────────────────────────────────────────────────────
    metrics = compute_metrics(df, cmd_col, actual_col, step_start)

    print(f"\n{'─' * 50}")
    print(f"  INNER LOOP METRICS — {axis.upper()}")
    print(f"{'─' * 50}")
    for key, value in metrics.items():
        if isinstance(value, float):
            print(f"  {key:25s}: {value:.4f}")
        else:
            print(f"  {key:25s}: {value}")
    print(f"{'─' * 50}")

    return metrics


def main():
    parser = argparse.ArgumentParser(description="Plot Test 1 results")
    parser.add_argument("--csv", type=str, default=None,
                        help="Path to the Test 1 CSV file")
    parser.add_argument("--axis", type=str, default=None,
                        choices=["vx", "vy", "vz", "yaw_rate"],
                        help="Axis to look for if --csv is not specified")
    args = parser.parse_args()

    if args.csv:
        csv_path = args.csv
    else:
        csv_path = find_latest_csv(args.axis)

    if csv_path is None or not os.path.exists(csv_path):
        print("ERROR: No Test 1 CSV found. Run test_1_inner_loop.py first.")
        print("  Or specify: --csv path/to/file.csv")
        sys.exit(1)

    plot_test_1(csv_path)


if __name__ == "__main__":
    main()
