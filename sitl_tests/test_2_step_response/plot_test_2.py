###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Post-processing for Test 2 — Outer Loop Step Response.
#         Reads the CSV produced by test_2_step_response.py and generates:
#           - Step response plot for each axis (e_x, e_y, e_area)
#           - Annotated Rise Time, Overshoot, Settling Time
#           - Drone trajectory vs. target position
#
# Usage:  python graphs_generation/plot_test_2.py
#         python graphs_generation/plot_test_2.py --csv graphs_generation/logs/test_2_step_response_yaw.csv
###############################################################################

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def find_latest_csv(axis=None):
    """Find the most recent Test 2 CSV file."""
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    if axis:
        pattern = os.path.join(logs_dir, f"test_2_{axis}.csv")
    else:
        pattern = os.path.join(logs_dir, "test_2_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def compute_step_metrics(time_s, signal, step_time, settling_pct=0.02):
    """
    Compute standard step response metrics.

    Args:
        time_s: Time array
        signal: Error signal (should approach 0 after the step response settles)
        step_time: Time when the step was applied
        settling_pct: Percentage band for settling time (default: 2%)

    Returns:
        dict with rise_time_s, settling_time_s, overshoot_pct, peak_value, final_value
    """
    # Get data after the step
    step_mask = time_s >= step_time
    t_step = time_s[step_mask] - step_time  # Time relative to step
    sig_step = signal[step_mask]

    if len(sig_step) < 10:
        return {}

    # Initial value (at the moment of the step)
    initial_value = sig_step[0]

    # Final value (average of last 20%)
    n_ss = max(1, len(sig_step) // 5)
    final_value = np.mean(sig_step[-n_ss:])

    # Response range
    response_range = final_value - initial_value
    if abs(response_range) < 1e-6:
        return {"initial_value": initial_value, "final_value": final_value}

    # Normalized response (0 → 1)
    normalized = (sig_step - initial_value) / response_range

    # Rise time: 10% → 90%
    t10_idx = np.where(normalized >= 0.1)[0]
    t90_idx = np.where(normalized >= 0.9)[0]
    if len(t10_idx) > 0 and len(t90_idx) > 0:
        rise_time_s = t_step[t90_idx[0]] - t_step[t10_idx[0]]
    else:
        rise_time_s = float('nan')

    # Peak and overshoot
    if response_range > 0:
        peak_idx = np.argmax(sig_step)
    else:
        peak_idx = np.argmin(sig_step)
    peak_value = sig_step[peak_idx]
    peak_time = t_step[peak_idx]

    if abs(response_range) > 1e-6:
        overshoot_pct = abs((peak_value - final_value) / response_range) * 100
    else:
        overshoot_pct = 0.0

    # Settling time: last time the signal exits the ±settling_pct band
    settling_band = abs(response_range) * settling_pct
    within_band = np.abs(sig_step - final_value) <= settling_band

    # Find the last time it leaves the band
    settling_time_s = float('nan')
    if np.any(within_band):
        # Walk backwards from the end to find where it enters the band for good
        for i in range(len(within_band) - 1, -1, -1):
            if not within_band[i]:
                settling_time_s = t_step[i + 1] if i + 1 < len(t_step) else t_step[i]
                break
        else:
            settling_time_s = t_step[0]  # Always within band

    return {
        "initial_value": initial_value,
        "final_value": final_value,
        "rise_time_s": rise_time_s,
        "settling_time_s": settling_time_s,
        "overshoot_pct": overshoot_pct,
        "peak_value": peak_value,
        "peak_time_since_step_s": peak_time
    }


def plot_test_2(csv_path, output_dir=None):
    """Generate Test 2 plots and compute metrics."""
    print(f"Reading: {csv_path}")
    df = pd.read_csv(csv_path)

    if output_dir is None:
        output_dir = os.path.dirname(csv_path)

    # Determine axis from filename
    basename = os.path.basename(csv_path)
    if "alt" in basename:
        axis = "alt"
    elif "dist" in basename:
        axis = "dist"
    else:
        axis = "yaw"

    time_s = df["time_s"].values

    # Find the step time (when phase changes from SETTLE to STEP)
    phase = df["phase"].values
    step_indices = np.where(phase == "STEP")[0]
    if len(step_indices) > 0:
        step_time = time_s[step_indices[0]]
    else:
        step_time = time_s[len(time_s) // 2]  # Fallback

    print(f"Step applied at t = {step_time:.2f}s")

    # ── Figure 1: Summary for the specific axis ───────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

    # Define which signals to analyze based on the axis
    if axis == "yaw":
        err_col, err_label = "e_x", ("Horizontal Error (e_x)", "tab:blue")
        pos_drone, pos_target, pos_label = "drone_x", "target_x", "North Position (X)"
        cmd_col, cmd_label = "omega_z_cmd", ("Yaw Rate Cmd (ωz)", "tab:green")
    elif axis == "alt":
        err_col, err_label = "e_y", ("Vertical Error (e_y)", "tab:blue")
        pos_drone, pos_target, pos_label = "drone_z", "target_z", "Down Position (Z)"
        cmd_col, cmd_label = "vz_cmd", ("Vertical Velocity Cmd (vz)", "tab:green")
    else: # dist
        err_col, err_label = "e_area", ("Distance Error (e_area)", "tab:blue")
        pos_drone, pos_target, pos_label = "drone_y", "target_y", "East Position (Y)"
        cmd_col, cmd_label = "vx_cmd", ("Forward Velocity Cmd (vx)", "tab:green")

    # Create plots directory
    if os.path.basename(output_dir) == "logs":
        plots_dir = os.path.join(os.path.dirname(output_dir), "plots")
    else:
        plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    
    plot_basename = os.path.splitext(basename)[0]

    # Subplot 1: Error Signal
    ax = axes[0]
    signal = df[err_col].values
    ax.plot(time_s, signal, color=err_label[1], linewidth=1.5, alpha=0.9, label=err_label[0])
    ax.axvline(x=step_time, color='red', linestyle='--', alpha=0.4, label="Step Trigger")
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.2)
    ax.set_ylabel(err_label[0], fontsize=11)
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(f"Test 2: Step Response — {axis.upper()} Step", fontsize=14, fontweight='bold')

    # Compute metrics for the primary error signal
    metrics = compute_step_metrics(time_s, signal, step_time)

    # Annotate settling time band if available
    if metrics.get("final_value") is not None and metrics.get("settling_time_s") is not None:
        fv = metrics["final_value"]
        rng = abs(metrics.get("initial_value", 0) - fv) if metrics.get("initial_value") is not None else abs(fv)
        if rng > 1e-6:
            band = rng * 0.02
            ax.axhline(y=fv + band, color='gray', linestyle=':', alpha=0.4)
            ax.axhline(y=fv - band, color='gray', linestyle=':', alpha=0.4)
            ax.fill_between(time_s, fv - band, fv + band, alpha=0.05, color='green')

    # Subplot 2: Position Tracking (Contextual based on axis)
    ax = axes[1]
    if axis == "yaw":
        # Plot Drone Yaw vs Target Bearing
        target_bearing = np.degrees(np.arctan2(df["target_y"] - df["drone_y"], df["target_x"] - df["drone_x"]))
        drone_yaw = df["drone_yaw_deg"]
        ax.plot(time_s, drone_yaw, color="tab:orange", linewidth=1.5, label="Drone Yaw", alpha=0.9)
        ax.plot(time_s, target_bearing, 'r--', linewidth=2, label="Target Bearing", alpha=0.8)
        ax.set_ylabel("Angle (deg)", fontsize=11)
    elif axis == "dist":
        # Plot Measured Distance to Target vs Desired Distance
        measured_dist = df["distance_to_target"]
        desired_dist = np.full_like(time_s, 0.6) # 0.6m stopping distance
        ax.plot(time_s, measured_dist, color="tab:orange", linewidth=1.5, label="Measured Distance to Target", alpha=0.9)
        ax.plot(time_s, desired_dist, 'r--', linewidth=2, label="Desired Stopping Distance", alpha=0.8)
        ax.set_ylabel("Distance (m)", fontsize=11)
    elif axis == "alt":
        # Plot Target Altitude vs Drone Altitude (NED down is negative, so invert for altitude)
        target_alt = -df["target_z"]
        drone_alt = -df["drone_z"]
        ax.plot(time_s, drone_alt, color="tab:orange", linewidth=1.5, label="Drone Altitude", alpha=0.9)
        ax.plot(time_s, target_alt, 'r--', linewidth=2, label="Target Altitude", alpha=0.8)
        ax.set_ylabel("Altitude (m)", fontsize=11)

    ax.axvline(x=step_time, color='red', linestyle='--', alpha=0.4)
    ax.legend(loc="upper right", fontsize=9)

    # Subplot 3: Velocity Command
    ax = axes[2]
    ax.plot(time_s, df[cmd_col], color=cmd_label[1], linewidth=1.5, label=cmd_label[0])
    ax.axvline(x=step_time, color='red', linestyle='--', alpha=0.4)
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.2)
    ax.set_ylabel(cmd_label[0], fontsize=11)
    ax.legend(loc="upper right", fontsize=9)
    axes[2].set_xlabel("Time (s)", fontsize=12)

    import matplotlib.ticker as ticker
    for ax in axes:
        # Increase tick label sizes
        ax.tick_params(axis='both', which='major', labelsize=12)
        
        # Turn on minor ticks and set specific spacing
        ax.minorticks_on()
        ax.xaxis.set_major_locator(ticker.MultipleLocator(5.0))  # Major tick every 5s
        ax.xaxis.set_minor_locator(ticker.MultipleLocator(1.0))  # Minor tick every 1s
        
        # Draw dark, clear grids
        ax.grid(which='major', color='black', alpha=0.4, linewidth=0.8)
        ax.grid(which='minor', color='gray', alpha=0.3, linestyle='--')
        
        # Tighten bounds
        ax.set_xlim([time_s.min(), time_s.max()])

    plt.tight_layout()
    plot_path = os.path.join(plots_dir, f"{plot_basename}_summary.pdf")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.savefig(plot_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')    
    print(f"Saved: {plot_path}")
    plt.close()

    # ── Print Metrics ───────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print(f"  STEP RESPONSE METRICS — {axis.upper()} STEP")
    print(f"{'=' * 60}")

    print(f"\n  {err_col}:")
    for key, value in metrics.items():
        if isinstance(value, float):
            print(f"    {key:25s}: {value:.4f}")
        else:
            print(f"    {key:25s}: {value}")

    print(f"\n{'=' * 60}")

    all_metrics = {err_col: metrics}

    return all_metrics


def main():
    parser = argparse.ArgumentParser(description="Plot Test 2 step response results")
    parser.add_argument("--csv", type=str, default=None,
                        help="Path to the Test 2 CSV file")
    parser.add_argument("--axis", type=str, default=None,
                        choices=["yaw", "alt", "dist"],
                        help="Axis to look for if --csv is not specified")
    args = parser.parse_args()

    if args.csv:
        csv_path = args.csv
    else:
        csv_path = find_latest_csv(args.axis)

    if csv_path is None or not os.path.exists(csv_path):
        print("ERROR: No Test 2 CSV found. Run test_2_step_response.py first.")
        print("  Or specify: --csv path/to/file.csv")
        sys.exit(1)

    plot_test_2(csv_path)


if __name__ == "__main__":
    main()
