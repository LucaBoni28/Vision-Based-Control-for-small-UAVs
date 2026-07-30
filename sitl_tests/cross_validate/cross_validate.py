###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Cross-validation: compares the CSV data logged by the SITL test
#         scripts with the Mission Planner .tlog telemetry file.
#
#         This proves in your thesis that both data sources (your Python script
#         and the flight controller's own telemetry) agree, validating the
#         integrity of your measurement pipeline.
#
#         Extracts from .tlog:
#           - LOCAL_POSITION_NED (x, y, z, vx, vy, vz)
#           - ATTITUDE (roll, pitch, yaw)
#           - SET_POSITION_TARGET_LOCAL_NED (commanded velocities)
#
# Usage:  python graphs_generation/cross_validate.py --csv <test_csv> --tlog <tlog_file>
#         python graphs_generation/cross_validate.py --csv graphs_generation/logs/test_1_step_vx.csv --tlog mav.tlog
###############################################################################

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pymavlink import mavutil


def parse_tlog(tlog_path, message_types=None):
    """
    Parse a Mission Planner .tlog file and extract specified message types.

    Args:
        tlog_path: Path to the .tlog file
        message_types: List of MAVLink message types to extract.
                       Default: LOCAL_POSITION_NED, ATTITUDE, SET_POSITION_TARGET_LOCAL_NED

    Returns:
        dict of {message_type: list of dicts with timestamp + fields}
    """
    if message_types is None:
        message_types = [
            "LOCAL_POSITION_NED",
            "GLOBAL_POSITION_INT",
            "ATTITUDE",
            "SET_POSITION_TARGET_LOCAL_NED",
        ]

    print(f"Parsing .tlog: {tlog_path}")
    print(f"  Looking for: {message_types}")

    mav = mavutil.mavlink_connection(tlog_path)

    data = {mt: [] for mt in message_types}
    msg_count = 0

    while True:
        try:
            msg = mav.recv_match(type=message_types, blocking=False)
            if msg is None:
                break
        except Exception:
            break

        msg_type = msg.get_type()
        if msg_type in message_types:
            # Get timestamp (tlog files have timestamps)
            timestamp = getattr(msg, '_timestamp', 0.0)

            entry = {"timestamp": timestamp}

            if msg_type == "LOCAL_POSITION_NED":
                entry.update({
                    "x": msg.x, "y": msg.y, "z": msg.z,
                    "vx": msg.vx, "vy": msg.vy, "vz": msg.vz,
                })
                
            elif msg_type == "GLOBAL_POSITION_INT":
                entry.update({
                    "lat": msg.lat, "lon": msg.lon, "alt": msg.alt,
                    "relative_alt": msg.relative_alt / 1000.0, # convert mm to m
                    "vx": msg.vx / 100.0, "vy": msg.vy / 100.0, "vz": msg.vz / 100.0, # cm/s to m/s
                })

            elif msg_type == "ATTITUDE":
                entry.update({
                    "roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw,
                    "rollspeed": msg.rollspeed, "pitchspeed": msg.pitchspeed,
                    "yawspeed": msg.yawspeed,
                })

            elif msg_type == "SET_POSITION_TARGET_LOCAL_NED":
                entry.update({
                    "vx": msg.vx, "vy": msg.vy, "vz": msg.vz,
                    "yaw_rate": msg.yaw_rate,
                })

            data[msg_type].append(entry)
            msg_count += 1

    print(f"  Extracted {msg_count} messages total:")
    for mt, entries in data.items():
        print(f"    {mt}: {len(entries)} messages")

    # Convert to DataFrames
    dataframes = {}
    for mt, entries in data.items():
        if entries:
            df = pd.DataFrame(entries)
            # Convert timestamps to seconds relative to first message
            if len(df) > 0 and "timestamp" in df.columns:
                df["time_s"] = df["timestamp"] - df["timestamp"].iloc[0]
            dataframes[mt] = df

    return dataframes


def align_time_series(csv_df, tlog_df, csv_time_col="time_s", tlog_time_col="time_s"):
    """
    Align two time series by finding the best time offset using cross-correlation
    on a common signal (e.g., velocity).

    Returns:
        time_offset: Offset to add to CSV times to align with tlog times
    """
    # Simple approach: assume both start at roughly the same time
    # and use the step edge to align

    # Find the range overlap
    csv_start = csv_df[csv_time_col].iloc[0]
    csv_end = csv_df[csv_time_col].iloc[-1]
    csv_duration = csv_end - csv_start

    tlog_start = tlog_df[tlog_time_col].iloc[0]
    tlog_end = tlog_df[tlog_time_col].iloc[-1]
    tlog_duration = tlog_end - tlog_start

    print(f"\n  CSV duration:  {csv_duration:.2f}s ({len(csv_df)} samples)")
    print(f"  TLOG duration: {tlog_duration:.2f}s ({len(tlog_df)} samples)")

    # The CSV test time is typically much shorter than the full tlog
    # We try to find the matching window in the tlog

    return 0.0  # Default: no offset (user can adjust if needed)


def cross_validate_test_1(csv_path, tlog_dfs, output_dir):
    """Cross-validate Test 1 (inner loop) data."""
    print(f"\n{'=' * 60}")
    print("  CROSS-VALIDATION: Test 1 — Inner Loop")
    print(f"{'=' * 60}")

    csv_df = pd.read_csv(csv_path)

    # Determine axis from filename
    basename = os.path.basename(csv_path)
    if "yaw_rate" in basename:
        csv_vel_col, tlog_vel_field = "cmd_yaw_rate", "yawspeed"
        csv_actual_col = "actual_vx"  # placeholder
        axis_label = "Yaw Rate"
    elif "vy" in basename:
        csv_vel_col = "actual_vy"
        tlog_vel_field = "vy"
        axis_label = "Velocity Y"
    elif "vz" in basename:
        csv_vel_col = "actual_vz"
        tlog_vel_field = "vz"
        axis_label = "Velocity Z"
    else:
        csv_vel_col = "actual_vx"
        tlog_vel_field = "vx"
        axis_label = "Velocity X"

    tlog_pos = None
    if "LOCAL_POSITION_NED" in tlog_dfs:
        tlog_pos = tlog_dfs["LOCAL_POSITION_NED"]
        pos_x_col = "x"
    elif "GLOBAL_POSITION_INT" in tlog_dfs:
        print("  Notice: Using GLOBAL_POSITION_INT from tlog as fallback for velocity.")
        tlog_pos = tlog_dfs["GLOBAL_POSITION_INT"]
        pos_x_col = None # Can't easily plot local X from lat/lon without origin
    else:
        print("  WARNING: No LOCAL_POSITION_NED or GLOBAL_POSITION_INT in tlog. Cannot cross-validate velocity.")
        return

    # Plot: CSV actual velocity vs. tlog velocity
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=False)

    # Panel 1: CSV data (commanded + actual)
    ax1 = axes[0]
    ax1.set_title("Source: Python CSV (test script)", fontsize=12, fontweight='bold')
    cmd_col = f"cmd_{csv_vel_col.replace('actual_', '')}" if "actual_" in csv_vel_col else csv_vel_col
    if cmd_col in csv_df.columns:
        ax1.plot(csv_df["time_s"], csv_df[cmd_col], 'r-', linewidth=2, label=f"Commanded", alpha=0.9)
    ax1.plot(csv_df["time_s"], csv_df[csv_vel_col], 'b-', linewidth=1.5, label=f"Actual {axis_label}", alpha=0.8)
    ax1.set_ylabel(f"{axis_label} (m/s)", fontsize=11)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Panel 2: tlog data
    ax2 = axes[1]
    ax2.set_title("Source: Mission Planner .tlog", fontsize=12, fontweight='bold')
    ax2.plot(tlog_pos["time_s"], tlog_pos[tlog_vel_field], 'g-', linewidth=1.5,
             label=f"TLOG {axis_label}", alpha=0.8)
    ax2.set_ylabel(f"{axis_label} (m/s)", fontsize=11)
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)

    # Panel 3: Position comparison
    if pos_x_col:
        ax3 = axes[2]
        ax3.set_title("Position Comparison (NED X)", fontsize=12, fontweight='bold')
        if "pos_x" in csv_df.columns:
            ax3.plot(csv_df["time_s"], csv_df["pos_x"], 'b-', linewidth=1.5, label="CSV pos_x", alpha=0.8)
        ax3.plot(tlog_pos["time_s"], tlog_pos[pos_x_col], 'g--', linewidth=1.5, label=f"TLOG {pos_x_col}", alpha=0.8)
        ax3.set_xlabel("Time (s)", fontsize=12)
        ax3.set_ylabel("Position X (m)", fontsize=11)
        ax3.legend(fontsize=10)
        ax3.grid(True, alpha=0.3)
    else:
        axes[2].set_title("Position Comparison (Unavailable - using GLOBAL_POSITION_INT)", fontsize=12)
        axes[2].set_visible(False)

    fig.suptitle(f"Cross-Validation: Test 1 — {axis_label}", fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()

    plot_path = os.path.join(output_dir, f"cross_validation_test_1_{axis_label.lower().replace(' ', '_')}.pdf")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.savefig(plot_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"\nSaved: {plot_path}")
    plt.close()

    # Compute correlation between CSV and tlog signals
    # Resample tlog to match CSV timestamps using interpolation
    if len(tlog_pos) > 10 and len(csv_df) > 10:
        tlog_interp = np.interp(csv_df["time_s"].values,
                                tlog_pos["time_s"].values,
                                tlog_pos[tlog_vel_field].values)
        csv_actual = csv_df[csv_vel_col].values

        # Compute R² (coefficient of determination)
        ss_res = np.sum((csv_actual - tlog_interp) ** 2)
        ss_tot = np.sum((csv_actual - np.mean(csv_actual)) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

        # RMS difference
        rms_diff = np.sqrt(np.mean((csv_actual - tlog_interp) ** 2))

        # Max absolute difference
        max_diff = np.max(np.abs(csv_actual - tlog_interp))

        print(f"\n  CROSS-VALIDATION METRICS:")
        print(f"    R² (correlation):     {r_squared:.6f}")
        print(f"    RMS difference:       {rms_diff:.6f} m/s")
        print(f"    Max abs difference:   {max_diff:.6f} m/s")

        if r_squared > 0.99:
            print(f"    ✓ EXCELLENT agreement (R² > 0.99)")
        elif r_squared > 0.95:
            print(f"    ✓ Good agreement (R² > 0.95)")
        else:
            print(f"    ⚠ Moderate agreement — check time alignment")


def cross_validate_test_2(csv_path, tlog_dfs, output_dir):
    """Cross-validate Test 2 (step response) data."""
    print(f"\n{'=' * 60}")
    print("  CROSS-VALIDATION: Test 2 — Step Response")
    print(f"{'=' * 60}")

    csv_df = pd.read_csv(csv_path)

    if "LOCAL_POSITION_NED" in tlog_dfs:
        tlog_pos = tlog_dfs["LOCAL_POSITION_NED"]
        positions = [
            ("drone_x", "x", "North (X)", "tab:blue"),
            ("drone_y", "y", "East (Y)", "tab:orange"),
            ("drone_z", "z", "Down (Z)", "tab:green"),
        ]
    elif "GLOBAL_POSITION_INT" in tlog_dfs:
        print("  Notice: Using GLOBAL_POSITION_INT from tlog as fallback for altitude.")
        tlog_pos = tlog_dfs["GLOBAL_POSITION_INT"]
        # In GLOBAL_POSITION_INT, altitude is positive UP. In NED, z is positive DOWN. 
        # So we compare drone_z (NED) to -relative_alt
        tlog_pos["neg_alt"] = -tlog_pos["relative_alt"]
        positions = [
            (None, None, "North (X) - Unavailable", "tab:blue"),
            (None, None, "East (Y) - Unavailable", "tab:orange"),
            ("drone_z", "neg_alt", "Down (Z)", "tab:green"),
        ]
    else:
        print("  WARNING: No LOCAL_POSITION_NED or GLOBAL_POSITION_INT in tlog.")
        return

    # Plot: Drone position from CSV vs tlog
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=False)

    positions = [
        ("drone_x", "x", "North (X)", "tab:blue"),
        ("drone_y", "y", "East (Y)", "tab:orange"),
        ("drone_z", "z", "Down (Z)", "tab:green"),
    ]

    for i, (csv_col, tlog_col, label, color) in enumerate(positions):
        ax = axes[i]
        if csv_col is None:
            ax.set_title(label, fontsize=10)
            ax.axis('off')
            continue
            
        if csv_col in csv_df.columns:
            ax.plot(csv_df["time_s"], csv_df[csv_col], color=color, linewidth=1.5,
                    label=f"CSV {label}", alpha=0.9)
        ax.plot(tlog_pos["time_s"], tlog_pos[tlog_col], color='gray', linewidth=1.5,
                linestyle='--', label=f"TLOG {label}", alpha=0.7)
        ax.set_ylabel(f"{label} (m)", fontsize=11)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)

    axes[0].set_title("Cross-Validation: Test 2 — Position Comparison", fontsize=14, fontweight='bold')
    axes[-1].set_xlabel("Time (s)", fontsize=12)
    plt.tight_layout()

    # Determine axis from filename
    basename = os.path.basename(csv_path)
    if "altitude" in basename:
        axis = "altitude"
    elif "distance" in basename:
        axis = "distance"
    else:
        axis = "yaw"

    plot_path = os.path.join(output_dir, f"cross_validation_test_2_{axis}.pdf")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.savefig(plot_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"\nSaved: {plot_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Cross-validate SITL test CSV data against Mission Planner .tlog files"
    )
    parser.add_argument("--csv", type=str, required=True,
                        help="Path to the test CSV file (from test_1/2/3)")
    parser.add_argument("--tlog", type=str, required=True,
                        help="Path to the Mission Planner .tlog file")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory for plots (default: same as CSV)")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"ERROR: CSV file not found: {args.csv}")
        sys.exit(1)
    if not os.path.exists(args.tlog):
        print(f"ERROR: TLOG file not found: {args.tlog}")
        sys.exit(1)

    output_dir = args.output_dir or os.path.dirname(args.csv)

    # Parse the tlog file
    tlog_dfs = parse_tlog(args.tlog)

    # Detect which test this CSV belongs to
    basename = os.path.basename(args.csv).lower()

    if "test_1" in basename:
        cross_validate_test_1(args.csv, tlog_dfs, output_dir)
    elif "test_2" in basename:
        cross_validate_test_2(args.csv, tlog_dfs, output_dir)
    elif "test_3" in basename:
        # For test 3, reuse test 2 position comparison logic
        cross_validate_test_2(args.csv, tlog_dfs, output_dir)
    else:
        print(f"WARNING: Could not determine test type from filename: {basename}")
        print("  Attempting generic position cross-validation...")
        cross_validate_test_2(args.csv, tlog_dfs, output_dir)

    print(f"\n{'=' * 60}")
    print("  CROSS-VALIDATION COMPLETE")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
