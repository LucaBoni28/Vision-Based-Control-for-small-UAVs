###############################################################################
# Author: Luca Boninsegna
# Date:   26/08/2026
# Descr:  Post-processing for Test 4 — Trajectory Scenario.
#         Reads CSVs produced by test_4_trajectory.py and generates:
#           - Top-down 2D path (drone vs. target)
#           - Altitude profile over time
#           - Tracking error time series (e_x, e_y, distance)
#           - Velocity command time series (ωz, vz, vx)
#           - Ideal vs. Noisy comparison (when both exist)
#           - Per-scenario summary metrics table
#
# Usage:  python sitl_tests/test_4_trajectory/plot_test_4.py
#         python sitl_tests/test_4_trajectory/plot_test_4.py --run run_001
#         python sitl_tests/test_4_trajectory/plot_test_4.py --csv path/to/file.csv
###############################################################################

import argparse
import glob
import math
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.patches as mpatches

# Add project root to sys.path to import config
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from sitl_tests.utils.sitl_utils import load_config


# ─── Styling Helpers ──────────────────────────────────────────────────────────

def style_axes(axes, time_col=None):
    """Apply consistent styling to a list of axes."""
    for ax in axes:
        ax.tick_params(axis='both', which='major', labelsize=11)
        ax.minorticks_on()
        ax.grid(which='major', color='black', alpha=0.35, linewidth=0.8)
        ax.grid(which='minor', color='gray', alpha=0.25, linestyle='--')
        ax.get_yaxis().get_major_formatter().set_useOffset(False)
        if time_col is not None:
            ax.xaxis.set_major_locator(ticker.MultipleLocator(5.0))
            ax.xaxis.set_minor_locator(ticker.MultipleLocator(1.0))
            ax.set_xlim([time_col.min(), time_col.max()])


def add_phase_shading(ax, time_s, phase, label_added=False):
    """Shade settle vs track phases."""
    settle_mask = phase == "SETTLE"
    if settle_mask.any():
        t_settle_end = time_s[settle_mask].max()
        ax.axvspan(time_s.min(), t_settle_end, alpha=0.06, color='orange', zorder=0)
        if not label_added:
            ax.axvline(x=t_settle_end, color='orange', linestyle='--', alpha=0.5, label='Trajectory Start')
            return True
    return label_added


# ─── Metrics Computation ─────────────────────────────────────────────────────

def compute_scenario_metrics(df):
    """Compute tracking metrics for a scenario CSV."""
    # Only use the TRACK phase (not SETTLE)
    track = df[df["phase"] == "TRACK"].copy()
    if len(track) < 10:
        return {}

    config = load_config()
    desired_dist = config.calibration.desired_stopping_distance_m

    # Tracking errors
    e_x = track["e_x"].values
    e_y = track["e_y"].values
    distance = track["distance_to_target"].values
    dist_error = distance - desired_dist

    # Velocity commands
    vx = track["vx_cmd"].values
    vz = track["vz_cmd"].values
    omega_z = track["omega_z_cmd"].values

    # Command smoothness (RMS of jerk = rate of change of command)
    dt = np.diff(track["time_s"].values)
    dt[dt == 0] = 1e-6  # Avoid division by zero
    vx_jerk = np.abs(np.diff(vx) / dt)
    vz_jerk = np.abs(np.diff(vz) / dt)
    omega_z_jerk = np.abs(np.diff(omega_z) / dt)

    # Dropout statistics
    detected = track["detected"].astype(str).values
    dropout_count = np.sum(detected == "False")
    id_switch_count = np.sum(track["id_switched"].astype(str).values == "True")

    return {
        "mean_e_x": np.mean(np.abs(e_x)),
        "max_e_x": np.max(np.abs(e_x)),
        "mean_e_y": np.mean(np.abs(e_y)),
        "max_e_y": np.max(np.abs(e_y)),
        "mean_dist_error": np.mean(np.abs(dist_error)),
        "max_dist_error": np.max(np.abs(dist_error)),
        "rms_vx": np.sqrt(np.mean(vx ** 2)),
        "rms_vz": np.sqrt(np.mean(vz ** 2)),
        "rms_omega_z": np.sqrt(np.mean(omega_z ** 2)),
        "mean_vx_jerk": np.mean(vx_jerk),
        "mean_vz_jerk": np.mean(vz_jerk),
        "mean_omega_z_jerk": np.mean(omega_z_jerk),
        "dropout_count": dropout_count,
        "id_switch_count": id_switch_count,
        "total_frames": len(track),
    }


# ─── Plot Generation ─────────────────────────────────────────────────────────

def plot_trajectory_2d(df, plots_dir, basename, desired_dist=None):
    """Plot 1: Top-down 2D path + Altitude profile."""
    fig, (ax_2d, ax_alt) = plt.subplots(1, 2, figsize=(16, 7))

    time_s = df["time_s"].values

    # Calculate desired drone position (Target Object shifted by desired_dist along LOS)
    d_dist = desired_dist if desired_dist is not None else 5.0
    dx = df["target_x"] - df["drone_x"]
    dy = df["target_y"] - df["drone_y"]
    dz = df["target_z"] - df["drone_z"]
    dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)
    dist_3d = np.where(dist_3d == 0, 1e-6, dist_3d)
    
    df["desired_drone_x"] = df["target_x"] - (dx / dist_3d) * d_dist
    df["desired_drone_y"] = df["target_y"] - (dy / dist_3d) * d_dist
    df["desired_drone_z"] = df["target_z"] - (dz / dist_3d) * d_dist

    # ── Top-down view (East vs North in NED) ──────────────────────────────
    # NED: X=North, Y=East. Plot with East on x-axis, North on y-axis.
    ax_2d.plot(df["target_y"], df["target_x"], 'r--', linewidth=2.5,
               alpha=0.8, label="Target Object", zorder=5)
    ax_2d.plot(df["desired_drone_y"], df["desired_drone_x"], 'g:', linewidth=2.0,
               alpha=0.7, label="Desired Drone Path", zorder=5)
    ax_2d.plot(df["drone_y"], df["drone_x"], 'tab:blue', linewidth=1.5,
               alpha=0.9, label="Actual Drone Path", zorder=6)

    # Start and end markers
    ax_2d.plot(df["target_y"].iloc[0], df["target_x"].iloc[0], 'r^',
               markersize=12, zorder=10, label="Target Start")
    ax_2d.plot(df["target_y"].iloc[-1], df["target_x"].iloc[-1], 'rs',
               markersize=10, zorder=10, label="Target End")
    ax_2d.plot(df["drone_y"].iloc[0], df["drone_x"].iloc[0], 'b^',
               markersize=12, zorder=10, label="Drone Start")
    ax_2d.plot(df["drone_y"].iloc[-1], df["drone_x"].iloc[-1], 'bs',
               markersize=10, zorder=10, label="Drone End")

    # Heading arrows every 5 seconds
    step = max(1, int(5.0 / (time_s[1] - time_s[0]))) if len(time_s) > 1 else 1
    for i in range(0, len(df), step):
        yaw_rad = math.radians(df["drone_yaw_deg"].iloc[i])
        dx = 0.3 * math.sin(yaw_rad)   # East component of heading
        dy = 0.3 * math.cos(yaw_rad)   # North component of heading
        ax_2d.annotate("", xy=(df["drone_y"].iloc[i] + dx, df["drone_x"].iloc[i] + dy),
                       xytext=(df["drone_y"].iloc[i], df["drone_x"].iloc[i]),
                       arrowprops=dict(arrowstyle="->", color="tab:blue", lw=1.5, alpha=0.5))

    ax_2d.set_xlabel("East (m)", fontsize=12)
    ax_2d.set_ylabel("North (m)", fontsize=12)
    ax_2d.set_aspect('equal')
    ax_2d.margins(0.15)
    ax_2d.legend(fontsize=9, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=2)
    scenario = df["scenario"].iloc[0] if "scenario" in df.columns else "unknown"
    noise_mode = df["noise_mode"].iloc[0] if "noise_mode" in df.columns else "?"
    ax_2d.set_title(f"Top-Down Path — {scenario} ({noise_mode})", fontsize=13, fontweight='bold')
    ax_2d.grid(True, alpha=0.3)

    # ── Altitude profile ──────────────────────────────────────────────────
    # NED: down is positive, so altitude = -z
    ax_alt.plot(time_s, -df["target_z"], 'r--', linewidth=2.5, alpha=0.8, label="Target Object Altitude")
    ax_alt.plot(time_s, -df["desired_drone_z"], 'g:', linewidth=2.0, alpha=0.7, label="Desired Drone Altitude")
    ax_alt.plot(time_s, -df["drone_z"], 'tab:blue', linewidth=1.5, alpha=0.9, label="Actual Drone Altitude")
    ax_alt.set_xlabel("Time (s)", fontsize=12)
    ax_alt.set_ylabel("Altitude (m)", fontsize=12)
    ax_alt.margins(0.1)
    ax_alt.legend(fontsize=9, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=2)
    ax_alt.set_title("Altitude Profile", fontsize=13, fontweight='bold')
    style_axes([ax_alt], time_s)

    plt.tight_layout()
    save_path = os.path.join(plots_dir, f"{basename}_trajectory.pdf")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.savefig(save_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"  Saved: {save_path}")
    plt.close()


def plot_trajectory_3d(df, plots_dir, basename, desired_dist=None, interactive=False):
    """Plot 1b: 3D path of target and drone."""
    fig = plt.figure(figsize=(10, 8))
    ax_3d = fig.add_subplot(111, projection='3d')

    if "desired_drone_x" not in df.columns:
        d_dist = desired_dist if desired_dist is not None else 5.0
        dx = df["target_x"] - df["drone_x"]
        dy = df["target_y"] - df["drone_y"]
        dz = df["target_z"] - df["drone_z"]
        dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)
        dist_3d = np.where(dist_3d == 0, 1e-6, dist_3d)
        df["desired_drone_x"] = df["target_x"] - (dx / dist_3d) * d_dist
        df["desired_drone_y"] = df["target_y"] - (dy / dist_3d) * d_dist
        df["desired_drone_z"] = df["target_z"] - (dz / dist_3d) * d_dist

    # NED: X=North, Y=East, Z=Down (so -Z is altitude)
    ax_3d.plot(df["target_y"], df["target_x"], -df["target_z"], 'r--', linewidth=2.5, alpha=0.8, label="Target Object")
    ax_3d.plot(df["desired_drone_y"], df["desired_drone_x"], -df["desired_drone_z"], 'g:', linewidth=2.0, alpha=0.7, label="Desired Drone Path")
    ax_3d.plot(df["drone_y"], df["drone_x"], -df["drone_z"], 'tab:blue', linewidth=1.5, alpha=0.9, label="Actual Drone Path")

    start_y = df["target_y"].iloc[0]
    start_x = df["target_x"].iloc[0]
    start_z = -df["target_z"].iloc[0]

    # Start and end markers
    ax_3d.plot([start_y], [start_x], [start_z], 'r^', markersize=8, label="Target Start")
    ax_3d.plot([df["target_y"].iloc[-1]], [df["target_x"].iloc[-1]], [-df["target_z"].iloc[-1]], 'rs', markersize=8, label="Target End")
    ax_3d.plot([df["drone_y"].iloc[0]], [df["drone_x"].iloc[0]], [-df["drone_z"].iloc[0]], 'b^', markersize=8, label="Drone Start")
    ax_3d.plot([df["drone_y"].iloc[-1]], [df["drone_x"].iloc[-1]], [-df["drone_z"].iloc[-1]], 'bs', markersize=8, label="Drone End")

    ax_3d.set_xlabel("East (m)", fontsize=10)
    ax_3d.set_ylabel("North (m)", fontsize=10)
    ax_3d.set_zlabel("Altitude (m)", fontsize=10)
    
    scenario = df["scenario"].iloc[0] if "scenario" in df.columns else "unknown"
    noise_mode = df["noise_mode"].iloc[0] if "noise_mode" in df.columns else "?"
    ax_3d.set_title(f"3D Path — {scenario} ({noise_mode})", fontsize=13, fontweight='bold')
    
    # Push legend outside to avoid occlusion
    ax_3d.legend(fontsize=9, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=2)

    plt.tight_layout()
    save_path = os.path.join(plots_dir, f"{basename}_trajectory_3d.pdf")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.savefig(save_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"  Saved: {save_path}")
    
    if interactive:
        print("  Showing interactive 3D plot (close the window to continue)...")
        plt.show()
    
    plt.close()


def plot_tracking_errors(df, plots_dir, basename, desired_dist=None):
    """Plot 2: Tracking error time series (3 subplots)."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

    time_s = df["time_s"].values
    phase = df["phase"].values

    config = load_config()
    if desired_dist is None:
        desired_dist = config.calibration.desired_stopping_distance_m

    # ── Subplot 1: Horizontal error (e_x) ────────────────────────────────
    ax = axes[0]
    ax.plot(time_s, df["e_x"], color='tab:blue', linewidth=1.5, alpha=0.9, label="e_x (horizontal)")
    if "e_x_clean" in df.columns:
        ax.plot(time_s, df["e_x_clean"], color='tab:cyan', linewidth=0.8, alpha=0.5, label="e_x (clean)")
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.2)
    label_added = add_phase_shading(ax, time_s, phase)
    ax.set_ylabel("e_x (normalized)", fontsize=11)
    ax.legend(loc="upper right", fontsize=9)
    scenario = df["scenario"].iloc[0] if "scenario" in df.columns else "unknown"
    noise_mode = df["noise_mode"].iloc[0] if "noise_mode" in df.columns else "?"
    ax.set_title(f"Test 4: Tracking Errors — {scenario} ({noise_mode})", fontsize=14, fontweight='bold')

    # ── Subplot 2: Vertical error (e_y) ──────────────────────────────────
    ax = axes[1]
    ax.plot(time_s, df["e_y"], color='tab:orange', linewidth=1.5, alpha=0.9, label="e_y (vertical)")
    if "e_y_clean" in df.columns:
        ax.plot(time_s, df["e_y_clean"], color='tab:red', linewidth=0.8, alpha=0.5, label="e_y (clean)")
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.2)
    add_phase_shading(ax, time_s, phase)
    ax.set_ylabel("e_y (normalized)", fontsize=11)
    ax.legend(loc="upper right", fontsize=9)

    # ── Subplot 3: Distance tracking ─────────────────────────────────────
    ax = axes[2]
    ax.plot(time_s, df["distance_to_target"], color='tab:green', linewidth=1.5,
            alpha=0.9, label="Measured Distance")
    if "distance_clean" in df.columns:
        ax.plot(time_s, df["distance_clean"], color='tab:olive', linewidth=0.8,
                alpha=0.5, label="Clean Distance")
    ax.axhline(y=desired_dist, color='red', linestyle='--', linewidth=2,
               alpha=0.7, label=f"Desired ({desired_dist:.1f}m)")
    add_phase_shading(ax, time_s, phase)
    ax.set_ylabel("Distance (m)", fontsize=11)
    ax.legend(loc="upper right", fontsize=9)
    ax.set_xlabel("Time (s)", fontsize=12)

    style_axes(axes, time_s)
    plt.tight_layout()
    save_path = os.path.join(plots_dir, f"{basename}_errors.pdf")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.savefig(save_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"  Saved: {save_path}")
    plt.close()


def plot_velocity_commands(df, plots_dir, basename):
    """Plot 3: Velocity command time series (3 subplots)."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

    time_s = df["time_s"].values
    phase = df["phase"].values

    cmd_configs = [
        ("omega_z_cmd", "Yaw Rate (ωz) [rad/s]", "tab:purple"),
        ("vz_cmd", "Vertical Vel (vz) [m/s]", "tab:red"),
        ("vx_cmd", "Forward Vel (vx) [m/s]", "tab:green"),
    ]

    for ax, (col, label, color) in zip(axes, cmd_configs):
        ax.plot(time_s, df[col], color=color, linewidth=1.2, alpha=0.9, label=label)
        ax.axhline(y=0, color='black', linestyle='-', alpha=0.2)
        add_phase_shading(ax, time_s, phase)
        ax.set_ylabel(label, fontsize=11)
        ax.legend(loc="upper right", fontsize=9)

    scenario = df["scenario"].iloc[0] if "scenario" in df.columns else "unknown"
    noise_mode = df["noise_mode"].iloc[0] if "noise_mode" in df.columns else "?"
    axes[0].set_title(f"Test 4: Velocity Commands — {scenario} ({noise_mode})", fontsize=14, fontweight='bold')
    axes[-1].set_xlabel("Time (s)", fontsize=12)

    style_axes(axes, time_s)
    plt.tight_layout()
    save_path = os.path.join(plots_dir, f"{basename}_commands.pdf")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.savefig(save_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"  Saved: {save_path}")
    plt.close()


def plot_comparison(df_ideal, df_noisy, plots_dir, scenario_name):
    """Plot 4: Overlay ideal vs noisy for comparison."""
    fig, axes = plt.subplots(3, 2, figsize=(18, 14))

    config = load_config()
    desired_dist = config.calibration.desired_stopping_distance_m

    # Left column: Tracking errors comparison
    error_data = [
        ("e_x", "Horizontal Error (e_x)"),
        ("e_y", "Vertical Error (e_y)"),
        ("distance_to_target", "Distance to Target (m)"),
    ]

    for i, (col, label) in enumerate(error_data):
        ax = axes[i, 0]
        t_ideal = df_ideal["time_s"].values
        t_noisy = df_noisy["time_s"].values

        ax.plot(t_ideal, df_ideal[col], color='tab:blue', linewidth=1.5,
                alpha=0.7, label="Ideal")
        ax.plot(t_noisy, df_noisy[col], color='tab:orange', linewidth=1.5,
                alpha=0.7, label="Noisy")
        ax.axhline(y=0, color='black', linestyle='-', alpha=0.2)

        if col == "distance_to_target":
            ax.axhline(y=desired_dist, color='red', linestyle='--', alpha=0.5,
                       label=f"Desired ({desired_dist:.1f}m)")

        ax.set_ylabel(label, fontsize=11)
        ax.legend(fontsize=9, loc='upper right')

    axes[0, 0].set_title(f"Tracking Errors — {scenario_name}", fontsize=13, fontweight='bold')
    axes[-1, 0].set_xlabel("Time (s)", fontsize=12)

    # Right column: Velocity commands comparison
    cmd_data = [
        ("omega_z_cmd", "Yaw Rate (ωz) [rad/s]"),
        ("vz_cmd", "Vertical Vel (vz) [m/s]"),
        ("vx_cmd", "Forward Vel (vx) [m/s]"),
    ]

    for i, (col, label) in enumerate(cmd_data):
        ax = axes[i, 1]
        t_ideal = df_ideal["time_s"].values
        t_noisy = df_noisy["time_s"].values

        ax.plot(t_ideal, df_ideal[col], color='tab:blue', linewidth=1.5,
                alpha=0.7, label="Ideal")
        ax.plot(t_noisy, df_noisy[col], color='tab:orange', linewidth=1.5,
                alpha=0.7, label="Noisy")
        ax.axhline(y=0, color='black', linestyle='-', alpha=0.2)
        ax.set_ylabel(label, fontsize=11)
        ax.legend(fontsize=9, loc='upper right')

    axes[0, 1].set_title(f"Velocity Commands — {scenario_name}", fontsize=13, fontweight='bold')
    axes[-1, 1].set_xlabel("Time (s)", fontsize=12)

    for row in axes:
        style_axes(row)

    plt.tight_layout()
    save_path = os.path.join(plots_dir, f"test_4_{scenario_name}_comparison.pdf")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.savefig(save_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"  Saved: {save_path}")
    plt.close()


def print_metrics_table(metrics_dict):
    """Print a comparison table of metrics for ideal vs noisy."""
    print(f"\n{'─' * 70}")
    print(f"  {'Metric':<30s} {'Ideal':>10s} {'Noisy':>10s} {'Δ':>10s}")
    print(f"{'─' * 70}")

    display_metrics = [
        ("mean_e_x", "Mean |e_x| (norm)", ".4f"),
        ("max_e_x", "Max |e_x| (norm)", ".4f"),
        ("mean_e_y", "Mean |e_y| (norm)", ".4f"),
        ("max_e_y", "Max |e_y| (norm)", ".4f"),
        ("mean_dist_error", "Mean |dist error| (m)", ".3f"),
        ("max_dist_error", "Max |dist error| (m)", ".3f"),
        ("rms_vx", "RMS vx_cmd (m/s)", ".3f"),
        ("rms_omega_z", "RMS ωz_cmd (rad/s)", ".3f"),
        ("mean_vx_jerk", "Mean vx jerk (m/s²)", ".3f"),
        ("mean_omega_z_jerk", "Mean ωz jerk (rad/s²)", ".3f"),
        ("dropout_count", "Detection dropouts", "d"),
        ("id_switch_count", "ID switches", "d"),
    ]

    ideal = metrics_dict.get("ideal", {})
    noisy = metrics_dict.get("noisy", {})

    for key, label, fmt in display_metrics:
        v_ideal = ideal.get(key, 0)
        v_noisy = noisy.get(key, 0)
        if isinstance(v_ideal, float) and v_ideal > 0:
            delta_pct = f"{((v_noisy - v_ideal) / v_ideal) * 100:+.0f}%"
        else:
            delta_pct = "—"
        print(f"  {label:<30s} {v_ideal:>10{fmt}} {v_noisy:>10{fmt}} {delta_pct:>10s}")

    print(f"{'─' * 70}")


# ─── File Discovery ──────────────────────────────────────────────────────────

def find_latest_run():
    """Find the most recent run directory."""
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    runs = glob.glob(os.path.join(logs_dir, "run_*"))
    if not runs:
        return None
    return max(runs, key=os.path.getmtime)


def find_csvs_in_run(run_dir):
    """Find all Test 4 CSVs in a run directory and group by scenario."""
    csvs = glob.glob(os.path.join(run_dir, "test_4_*.csv"))
    scenarios = {}
    for csv_path in csvs:
        basename = os.path.basename(csv_path)
        # Parse: test_4_<scenario>_<mode>.csv
        parts = basename.replace("test_4_", "").replace(".csv", "")
        # Find mode (last part after last underscore)
        if parts.endswith("_ideal"):
            scenario = parts[:-6]
            mode = "ideal"
        elif parts.endswith("_noisy"):
            scenario = parts[:-6]
            mode = "noisy"
        else:
            scenario = parts
            mode = "unknown"

        if scenario not in scenarios:
            scenarios[scenario] = {}
        scenarios[scenario][mode] = csv_path

    return scenarios


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Plot Test 4 trajectory scenario results")
    parser.add_argument("--csv", type=str, default=None,
                        help="Path to a specific Test 4 CSV file")
    parser.add_argument("--run", type=str, default=None,
                        help="Run name (e.g., run_001) to process")
    parser.add_argument("--desired-distance", type=float, default=None,
                        help="Override desired stopping distance for plots")
    args = parser.parse_args()

    config = load_config()
    desired_dist = args.desired_distance or config.calibration.desired_stopping_distance_m

    if args.csv:
        # Single CSV mode
        csv_path = args.csv
        if not os.path.exists(csv_path):
            print(f"ERROR: File not found: {csv_path}")
            sys.exit(1)

        df = pd.read_csv(csv_path)
        basename = os.path.splitext(os.path.basename(csv_path))[0]
        run_folder = os.path.basename(os.path.dirname(csv_path))
        plots_dir = os.path.abspath(os.path.join(os.path.dirname(csv_path), "..", "..", "plots", run_folder))
        os.makedirs(plots_dir, exist_ok=True)

        print(f"Processing: {csv_path}")
        plot_trajectory_2d(df, plots_dir, basename, desired_dist)
        plot_trajectory_3d(df, plots_dir, basename, desired_dist=desired_dist, interactive=True)
        plot_tracking_errors(df, plots_dir, basename, desired_dist)
        plot_velocity_commands(df, plots_dir, basename)

        metrics = compute_scenario_metrics(df)
        print(f"\n{'-' * 50}")
        print(f"  METRICS - {basename}")
        print(f"{'-' * 50}")
        for key, value in metrics.items():
            if isinstance(value, float):
                print(f"  {key:<25s}: {value:.4f}")
            else:
                print(f"  {key:<25s}: {value}")
        print(f"{'-' * 50}")
        return

    # Run directory mode
    if args.run:
        run_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", args.run)
    else:
        run_dir = find_latest_run()

    if run_dir is None or not os.path.exists(run_dir):
        print("ERROR: No run directory found. Run test_4_trajectory.py first.")
        print("  Or specify: --run run_001  or  --csv path/to/file.csv")
        sys.exit(1)

    print(f"Processing run: {run_dir}")
    run_folder = os.path.basename(run_dir)
    plots_dir = os.path.join(os.path.dirname(run_dir), "..", "plots", run_folder)
    os.makedirs(plots_dir, exist_ok=True)

    scenarios = find_csvs_in_run(run_dir)

    if not scenarios:
        print(f"ERROR: No Test 4 CSVs found in {run_dir}")
        sys.exit(1)

    print(f"Found {len(scenarios)} scenario(s): {list(scenarios.keys())}")

    for scenario_name, mode_files in scenarios.items():
        print(f"\n{'=' * 60}")
        print(f"  Scenario: {scenario_name}")
        print(f"  Available: {list(mode_files.keys())}")
        print(f"{'=' * 60}")

        # Generate individual plots for each mode
        all_metrics = {}
        for mode, csv_path in mode_files.items():
            df = pd.read_csv(csv_path)
            basename = os.path.splitext(os.path.basename(csv_path))[0]

            print(f"\n  [{mode.upper()}] {csv_path}")
            plot_trajectory_2d(df, plots_dir, basename, desired_dist)
            plot_trajectory_3d(df, plots_dir, basename, desired_dist=desired_dist)
            plot_tracking_errors(df, plots_dir, basename, desired_dist)
            plot_velocity_commands(df, plots_dir, basename)

            metrics = compute_scenario_metrics(df)
            all_metrics[mode] = metrics

        # Generate comparison plot if both ideal and noisy exist
        if "ideal" in mode_files and "noisy" in mode_files:
            print(f"\n  Generating comparison plots...")
            df_ideal = pd.read_csv(mode_files["ideal"])
            df_noisy = pd.read_csv(mode_files["noisy"])
            plot_comparison(df_ideal, df_noisy, plots_dir, scenario_name)
            print_metrics_table(all_metrics)

    print(f"\n{'=' * 60}")
    print(f"  All plots saved to: {plots_dir}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
