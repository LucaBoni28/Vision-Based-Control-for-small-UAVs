###############################################################################
# Author: Luca Boninsegna
# Date:   23/07/26
# Descr:  Comprehensive visualization of combined telemetry & hardware variables
#         Generates thesis-ready graphs with descriptive filenames.
###############################################################################

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Configure academic formatting for LaTeX/PDF integration
plt.rcParams.update({
    'font.family': 'serif', 
    'font.size': 12
})

import argparse

parser = argparse.ArgumentParser(description="Telemetry Plotter")
parser.add_argument('--run-name', default=None, help='Subfolder name for grouping logs (e.g. run_002)')
parser.add_argument('--target-dist', type=float, default=0.7, help='Target hover distance in meters for the plot')
args = parser.parse_args()

log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs')
plot_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'plots')

if args.run_name:
    log_dir = os.path.join(log_dir, args.run_name)
    plot_dir = os.path.join(plot_dir, args.run_name)

os.makedirs(plot_dir, exist_ok=True)

# Define algorithms
algorithms = ['bytetrack', 'botsort', 'deepsort']
colors = ['darkred', 'darkblue', 'darkgreen']
pretty_names = {'bytetrack': 'ByteTrack', 'botsort': 'BoT-SORT', 'deepsort': 'DeepSORT'}

# =============================================================================
# 1. HARDWARE STATISTICS PLOTTING (GPU, DLA, CPU, RAM)
# =============================================================================
fig_hw, axs_hw = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

hw_found = False
for algo_name, color in zip(algorithms, colors):
    filename = os.path.join(log_dir, f'{algo_name}_hardware_stats.csv')
    try:
        df_hw = pd.read_csv(filename) 
        p_name = pretty_names[algo_name]
        if 'GPU_Util_%' in df_hw.columns:
            avg_gpu = df_hw['GPU_Util_%'].mean()
            axs_hw[0].plot(df_hw['Time_Sec'], df_hw['GPU_Util_%'], color=color, linewidth=1.5, alpha=0.8, label=f"{p_name} (Avg: {avg_gpu:.1f}%)")
        
        avg_cpu = df_hw['CPU_Util_%'].mean()
        axs_hw[1].plot(df_hw['Time_Sec'], df_hw['CPU_Util_%'], color=color, linewidth=1.5, alpha=0.8, label=f"{p_name} (Avg: {avg_cpu:.1f}%)")
        
        avg_ram = df_hw['RAM_Usage_%'].mean()
        axs_hw[2].plot(df_hw['Time_Sec'], df_hw['RAM_Usage_%'], color=color, linewidth=1.5, alpha=0.8, label=f"{p_name} (Avg: {avg_ram:.2f})")
        hw_found = True
    except FileNotFoundError:
        print(f"Notice: {filename} not found.")

if hw_found:
    axs_hw[0].set_ylabel('GPU Util [%]')
    axs_hw[0].set_title('Hardware Utilization Comparison Across Tracking Algorithms')
    axs_hw[0].grid(True, linestyle=':', alpha=0.7)
    axs_hw[0].legend(loc='upper right', fontsize=10)

    axs_hw[1].set_ylabel('CPU Util [%]')
    axs_hw[1].grid(True, linestyle=':', alpha=0.7)
    axs_hw[1].legend(loc='upper right', fontsize=10)

    axs_hw[2].set_xlabel('Time [s]')
    axs_hw[2].set_ylabel('RAM Usage [%]')
    axs_hw[2].grid(True, linestyle=':', alpha=0.7)
    axs_hw[2].legend(loc='upper right', fontsize=10)

    plt.figure(fig_hw.number)
    plt.tight_layout()
    out_path = os.path.join(plot_dir, 'hardware_comparison_stats.png')
    plt.savefig(out_path, format='png', dpi=300)
    print(f"Hardware stats plot saved: {out_path}")

# =============================================================================
# 2. COMBINED TRACKING BENCHMARK ANALYSIS
#    - 2a: Average bar charts (split into 2 side-by-side images)
#    - 2b: Time-series line plots (2 additional side-by-side images)
# =============================================================================
avg_fps = {}
id_switches = {}
avg_latency = {}
avg_jitter = {}
trackers_found = []
benchmark_dfs = {}  # Store DataFrames for time-series plots

for algo_name in algorithms:
    filepath = os.path.join(log_dir, f'combined_{algo_name}.csv')
    try:
        df_bench = pd.read_csv(filepath)
        p_name = pretty_names[algo_name]
        trackers_found.append(p_name)
        benchmark_dfs[algo_name] = df_bench

        # Average FPS from processing time
        avg_proc_ms = df_bench['Processing_Time_ms'].mean()
        avg_fps[p_name] = 1000.0 / avg_proc_ms if avg_proc_ms > 0 else 0
        
        # Average Latency
        avg_lat = df_bench['Pipeline_Latency_ms'].mean()
        avg_latency[p_name] = avg_lat

        # Count ID switches: number of times Object_ID changes (excluding -1 gaps)
        valid_ids = df_bench[df_bench['Object_ID'] != -1]['Object_ID']
        switches = (valid_ids != valid_ids.shift()).sum() - 1  # first entry is not a switch
        id_switches[p_name] = max(0, int(switches))
        
        # Calculate Bounding Box Jitter (average pixel movement per frame when target is found)
        valid_boxes = df_bench[df_bench['Object_ID'] != -1]
        if len(valid_boxes) > 1:
            diff_x = valid_boxes['Bbox_X'].diff().abs().mean()
            diff_y = valid_boxes['Bbox_Y'].diff().abs().mean()
            avg_jitter[p_name] = diff_x + diff_y
        else:
            avg_jitter[p_name] = 0

    except FileNotFoundError:
        print(f"Notice: {filepath} not found.")

colors_bench = ['#2ecc71', '#3498db', '#e74c3c']

# --- 2a: Average Bar Charts (side-by-side) ---
if trackers_found:
    x_pos = range(len(trackers_found))

    # Image 1: FPS | Latency (side-by-side)
    fig_avg1, (ax_fps, ax_lat) = plt.subplots(1, 2, figsize=(12, 5))

    bars_fps = ax_fps.bar(x_pos, [avg_fps[t] for t in trackers_found],
                          color=colors_bench[:len(trackers_found)], width=0.5)
    ax_fps.set_xticks(list(x_pos))
    ax_fps.set_xticklabels(trackers_found)
    ax_fps.set_ylabel('Average FPS')
    ax_fps.set_title('Algorithm FPS Comparison')
    ax_fps.grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars_fps, [avg_fps[t] for t in trackers_found]):
        ax_fps.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
                    f'{val:.1f}', ha='center', va='bottom', fontweight='bold')

    bars_lat = ax_lat.bar(x_pos, [avg_latency[t] for t in trackers_found],
                          color=colors_bench[:len(trackers_found)], width=0.5)
    ax_lat.set_xticks(list(x_pos))
    ax_lat.set_xticklabels(trackers_found)
    ax_lat.set_ylabel('Pipeline Latency [ms]')
    ax_lat.set_title('Pipeline Latency Comparison')
    ax_lat.grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars_lat, [avg_latency[t] for t in trackers_found]):
        ax_lat.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
                    f'{val:.1f}', ha='center', va='bottom', fontweight='bold')

    plt.figure(fig_avg1.number)
    plt.tight_layout()
    out_path = os.path.join(plot_dir, 'benchmark_avg_fps_latency.png')
    plt.savefig(out_path, format='png', dpi=300)
    print(f"Benchmark averages (FPS & Latency) saved: {out_path}")

    # Image 2: ID Switches | Jitter (side-by-side)
    fig_avg2, (ax_ids, ax_jit) = plt.subplots(1, 2, figsize=(12, 5))

    bars_ids = ax_ids.bar(x_pos, [id_switches[t] for t in trackers_found],
                          color=colors_bench[:len(trackers_found)], width=0.5)
    ax_ids.set_xticks(list(x_pos))
    ax_ids.set_xticklabels(trackers_found)
    ax_ids.set_ylabel('ID Switches')
    ax_ids.set_title('Tracking ID Switch Comparison')
    ax_ids.grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars_ids, [id_switches[t] for t in trackers_found]):
        ax_ids.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.1,
                    f'{val}', ha='center', va='bottom', fontweight='bold')

    bars_jit = ax_jit.bar(x_pos, [avg_jitter[t] for t in trackers_found],
                          color=colors_bench[:len(trackers_found)], width=0.5)
    ax_jit.set_xticks(list(x_pos))
    ax_jit.set_xticklabels(trackers_found)
    ax_jit.set_ylabel('Average Jitter [pixels]')
    ax_jit.set_title('Bounding Box Jitter')
    ax_jit.grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars_jit, [avg_jitter[t] for t in trackers_found]):
        ax_jit.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
                    f'{val:.1f}', ha='center', va='bottom', fontweight='bold')

    plt.figure(fig_avg2.number)
    plt.tight_layout()
    out_path = os.path.join(plot_dir, 'benchmark_avg_id_switches_jitter.png')
    plt.savefig(out_path, format='png', dpi=300)
    print(f"Benchmark averages (ID Switches & Jitter) saved: {out_path}")

# --- 2b: Time-Series Line Plots (side-by-side) ---
if benchmark_dfs:
    # Image 3: FPS vs Time | Latency vs Time (side-by-side)
    fig_ts1, (ax_fps_ts, ax_lat_ts) = plt.subplots(1, 2, figsize=(14, 5))

    for algo_name, color in zip(algorithms, colors):
        if algo_name not in benchmark_dfs:
            continue
        df = benchmark_dfs[algo_name]
        p_name = pretty_names[algo_name]

        # Instantaneous FPS = 1000 / Processing_Time_ms
        inst_fps = 1000.0 / df['Processing_Time_ms'].replace(0, np.nan)
        ax_fps_ts.plot(df['Time_Sec'], inst_fps, color=color, linewidth=1, alpha=0.7, label=p_name)
        ax_lat_ts.plot(df['Time_Sec'], df['Pipeline_Latency_ms'], color=color, linewidth=1, alpha=0.7, label=p_name)

    ax_fps_ts.set_xlabel('Time [s]')
    ax_fps_ts.set_ylabel('FPS')
    ax_fps_ts.set_title('Instantaneous FPS Over Time')
    ax_fps_ts.grid(True, linestyle=':', alpha=0.7)
    ax_fps_ts.legend(loc='upper right', fontsize=10)

    ax_lat_ts.set_xlabel('Time [s]')
    ax_lat_ts.set_ylabel('Pipeline Latency [ms]')
    ax_lat_ts.set_title('Pipeline Latency Over Time')
    ax_lat_ts.grid(True, linestyle=':', alpha=0.7)
    ax_lat_ts.legend(loc='upper right', fontsize=10)

    plt.figure(fig_ts1.number)
    plt.tight_layout()
    out_path = os.path.join(plot_dir, 'benchmark_timeseries_fps_latency.png')
    plt.savefig(out_path, format='png', dpi=300)
    print(f"Benchmark time-series (FPS & Latency) saved: {out_path}")

    # Image 4: Cumulative ID Switches vs Time | Jitter vs Time (side-by-side)
    fig_ts2, (ax_ids_ts, ax_jit_ts) = plt.subplots(1, 2, figsize=(14, 5))

    for algo_name, color in zip(algorithms, colors):
        if algo_name not in benchmark_dfs:
            continue
        df = benchmark_dfs[algo_name]
        p_name = pretty_names[algo_name]

        # Cumulative ID switches over time
        valid_mask = df['Object_ID'] != -1
        id_changed = (df['Object_ID'] != df['Object_ID'].shift()) & valid_mask
        cumulative_switches = id_changed.cumsum()
        ax_ids_ts.plot(df['Time_Sec'], cumulative_switches, color=color, linewidth=1.5, label=p_name)

        # Instantaneous bounding box jitter (pixel movement per frame)
        jitter = df['Bbox_X'].diff().abs() + df['Bbox_Y'].diff().abs()
        jitter = jitter.where(valid_mask, other=np.nan)
        ax_jit_ts.plot(df['Time_Sec'], jitter, color=color, linewidth=1, alpha=0.6, label=p_name)

    ax_ids_ts.set_xlabel('Time [s]')
    ax_ids_ts.set_ylabel('Cumulative ID Switches')
    ax_ids_ts.set_title('ID Switches Over Time')
    ax_ids_ts.grid(True, linestyle=':', alpha=0.7)
    ax_ids_ts.legend(loc='upper left', fontsize=10)

    ax_jit_ts.set_xlabel('Time [s]')
    ax_jit_ts.set_ylabel('Jitter [pixels]')
    ax_jit_ts.set_title('Bounding Box Jitter Over Time')
    ax_jit_ts.grid(True, linestyle=':', alpha=0.7)
    ax_jit_ts.legend(loc='upper right', fontsize=10)

    plt.figure(fig_ts2.number)
    plt.tight_layout()
    out_path = os.path.join(plot_dir, 'benchmark_timeseries_id_switches_jitter.png')
    plt.savefig(out_path, format='png', dpi=300)
    print(f"Benchmark time-series (ID Switches & Jitter) saved: {out_path}")

# =============================================================================
# 3. TELEMETRY PLOTS (Velocities, Distances, Areas per algorithm)
# =============================================================================
for algo_name in algorithms:
    filepath = os.path.join(log_dir, f'combined_{algo_name}.csv')
    try:
        df_dist = pd.read_csv(filepath)
        df_dist = df_dist[df_dist['Distance_Est'] > 0]
        if df_dist.empty:
            continue
        fig_dist, axs_dist = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
        p_name = pretty_names[algo_name]

        # 1. Translational Velocities
        axs_dist[0].plot(df_dist['Time_Sec'], df_dist['v_x'], color='blue', linewidth=1.5, label='Forward Vel ($v_x$)')
        axs_dist[0].plot(df_dist['Time_Sec'], df_dist['v_z'], color='purple', linewidth=1.5, label='Vertical Vel ($v_z$)')
        axs_dist[0].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8)
        axs_dist[0].set_ylabel('Velocity [m/s]')
        axs_dist[0].set_title(f'UAV Telemetry — {p_name}')
        axs_dist[0].grid(True, linestyle=':', alpha=0.7)
        axs_dist[0].legend(loc='upper right')

        # 2. Yaw Rate
        axs_dist[1].plot(df_dist['Time_Sec'], df_dist['omega_z'], color='orange', linewidth=1.5, label='Yaw Rate ($\\omega_z$)')
        axs_dist[1].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8)
        axs_dist[1].set_ylabel('Ang. Vel [rad/s]')
        axs_dist[1].grid(True, linestyle=':', alpha=0.7)
        axs_dist[1].legend(loc='upper right')

        # 3. Distance Estimation
        axs_dist[2].plot(df_dist['Time_Sec'], df_dist['Distance_Est'], color='red', linewidth=1.5, label='Estimated Distance')
        axs_dist[2].axhline(args.target_dist, color='green', linestyle='--', linewidth=2, label=f'Target Hover Threshold ({args.target_dist}m)')
        axs_dist[2].set_ylabel('Distance [m]')
        axs_dist[2].grid(True, linestyle=':', alpha=0.7)
        axs_dist[2].legend(loc='upper right')

        # 4. Bounding Box Area
        axs_dist[3].plot(df_dist['Time_Sec'], df_dist['A_real'], color='teal', linewidth=1.5, label='Bounding Box Area')
        axs_dist[3].set_ylabel('Area [px$^2$]')
        axs_dist[3].grid(True, linestyle=':', alpha=0.7)
        axs_dist[3].legend(loc='lower right')

        # 5. Pitch Angle
        pitch_deg = df_dist['current_pitch_rad'] * 180.0 / np.pi
        axs_dist[4].plot(df_dist['Time_Sec'], pitch_deg, color='brown', linewidth=1.5, label='Pitch Angle')
        axs_dist[4].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8)
        axs_dist[4].set_ylabel('Pitch [deg]')
        axs_dist[4].set_xlabel('Time [s]')
        axs_dist[4].grid(True, linestyle=':', alpha=0.7)
        axs_dist[4].legend(loc='upper right')

        plt.figure(fig_dist.number)
        plt.tight_layout()
        out_path = os.path.join(plot_dir, f'telemetry_velocities_distance_{algo_name}.png')
        plt.savefig(out_path, format='png', dpi=300)
        print(f"Telemetry plot saved for {p_name}: {out_path}")
    except FileNotFoundError:
        pass


# =============================================================================
# 4. STEP RESPONSE ANALYSIS (Per-axis: Yaw, Altitude, Distance)
#    Generates separate graphs with metrics stamped on the image.
# =============================================================================

# Define which axes to look for and how to plot them
STEP_AXES = {
    'yaw': {
        'error_col': 'e_x',
        'velocity_col': 'omega_z',
        'error_label': '$e_x$ (Horizontal Error)',
        'velocity_label': '$\\omega_z$ (Yaw Rate)',
        'error_ylabel': 'Normalized Error $e_x$',
        'velocity_ylabel': 'Yaw Rate [rad/s]',
        'title': 'Yaw PD Controller Step Response',
    },
    'altitude': {
        'error_col': 'e_y',
        'velocity_col': 'v_z',
        'error_label': '$e_y$ (Vertical Error)',
        'velocity_label': '$v_z$ (Vertical Velocity)',
        'error_ylabel': 'Normalized Error $e_y$',
        'velocity_ylabel': 'Vertical Velocity [m/s]',
        'title': 'Altitude PD Controller Step Response',
    },
    'distance': {
        'error_col': 'e_area',
        'velocity_col': 'v_x',
        'error_label': '$e_{area}$ (Area Error)',
        'velocity_label': '$v_x$ (Forward Velocity)',
        'error_ylabel': 'Normalized Error $e_{area}$',
        'velocity_ylabel': 'Forward Velocity [m/s]',
        'title': 'Distance PD Controller Step Response',
    },
}

# Try to load the step response CSV
step_csv = os.path.join(log_dir, 'step_response.csv')
try:
    df_step = pd.read_csv(step_csv)

    for axis_name, cfg in STEP_AXES.items():
        error_col = cfg['error_col']
        vel_col = cfg['velocity_col']

        # Skip this axis if the error column doesn't exist or is all zeros/empty
        if error_col not in df_step.columns:
            continue
        error_data = pd.to_numeric(df_step[error_col], errors='coerce')
        if error_data.dropna().empty or (error_data.abs() < 1e-9).all():
            continue

        fig_sr, axs_sr = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # --- Top subplot: Error vs Time ---
        axs_sr[0].plot(df_step['Time_Sec'], error_data, color='blue', linewidth=1.5, label=cfg['error_label'])
        axs_sr[0].axhline(0, color='black', linestyle='-', linewidth=0.5)

        # Settling band (+/- 2% of step size)
        step_size = error_data.iloc[0]  # First value = step magnitude
        settling_band = 0.02 * abs(step_size)
        axs_sr[0].axhspan(-settling_band, settling_band, alpha=0.15, color='green', label=f'$\\pm$2% Settling Band')

        # Overshoot
        min_ex = error_data.min()
        if min_ex < 0:
            overshoot_pct = abs(min_ex) / abs(step_size) * 100
            overshoot_idx = error_data.idxmin()
            axs_sr[0].annotate(f'Overshoot: {overshoot_pct:.1f}%',
                             xy=(df_step['Time_Sec'].iloc[overshoot_idx], min_ex),
                             xytext=(df_step['Time_Sec'].iloc[overshoot_idx] + 0.5, min_ex - 0.05),
                             arrowprops=dict(arrowstyle='->', color='red'),
                             color='red', fontsize=10)
        else:
            overshoot_pct = 0.0

        # Settling time (last time the signal leaves the settling band)
        settled = error_data.abs() <= settling_band
        settling_time = df_step['Time_Sec'].iloc[-1]
        for i in range(len(settled) - 1, -1, -1):
            if not settled.iloc[i]:
                if i + 1 < len(settled):
                    settling_time = df_step['Time_Sec'].iloc[i + 1]
                break
        axs_sr[0].axvline(settling_time, color='orange', linestyle='--', linewidth=1.5,
                         label=f'Settling Time: {settling_time:.2f}s')

        # Rising time (10% to 90% of the step)
        target_val = 0  # Final desired value for the error
        rise_10 = 0.9 * abs(step_size)  # Error has dropped to 90% of initial (10% of way there)
        rise_90 = 0.1 * abs(step_size)  # Error has dropped to 10% of initial (90% of way there)
        t_10, t_90 = None, None
        for idx in range(len(error_data)):
            if t_10 is None and abs(error_data.iloc[idx]) <= rise_10:
                t_10 = df_step['Time_Sec'].iloc[idx]
            if t_90 is None and abs(error_data.iloc[idx]) <= rise_90:
                t_90 = df_step['Time_Sec'].iloc[idx]
        rising_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else float('nan')

        # RMSE
        rmse = np.sqrt(np.mean(error_data ** 2))

        # Damping ratio from overshoot percentage
        if overshoot_pct > 0:
            ln_os = np.log(overshoot_pct / 100)
            damping_ratio = -ln_os / np.sqrt(np.pi**2 + ln_os**2)
        else:
            damping_ratio = 1.0  # Critically damped or over-damped

        # Stamp metrics directly on the image
        textstr = (f'Overshoot: {overshoot_pct:.1f}%\n'
                   f'Settling Time: {settling_time:.2f} s\n'
                   f'Rising Time: {rising_time:.2f} s\n'
                   f'RMSE: {rmse:.4f}\n'
                   f'Damping Ratio $\\zeta$: {damping_ratio:.3f}')
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        axs_sr[0].text(0.98, 0.98, textstr, transform=axs_sr[0].transAxes, fontsize=10,
                     verticalalignment='top', horizontalalignment='right', bbox=props)

        axs_sr[0].set_ylabel(cfg['error_ylabel'])
        axs_sr[0].set_title(cfg['title'])
        axs_sr[0].grid(True, linestyle=':', alpha=0.7)
        axs_sr[0].legend(loc='upper left')

        # --- Bottom subplot: Control Effort ---
        if vel_col in df_step.columns:
            vel_data = pd.to_numeric(df_step[vel_col], errors='coerce')
            axs_sr[1].plot(df_step['Time_Sec'], vel_data, color='orange', linewidth=1.5, label=cfg['velocity_label'])
        axs_sr[1].axhline(0, color='black', linestyle='-', linewidth=0.5)
        axs_sr[1].set_xlabel('Time [s]')
        axs_sr[1].set_ylabel(cfg['velocity_ylabel'])
        axs_sr[1].set_title('Control Effort')
        axs_sr[1].grid(True, linestyle=':', alpha=0.7)
        axs_sr[1].legend(loc='upper right')

        plt.figure(fig_sr.number)
        plt.tight_layout()
        out_path = os.path.join(plot_dir, f'step_response_{axis_name}.png')
        plt.savefig(out_path, format='png', dpi=300)
        print(f"Step response ({axis_name}) saved: {out_path}  |  "
              f"RMSE={rmse:.4f}, Overshoot={overshoot_pct:.1f}%, "
              f"Settling={settling_time:.2f}s, Rising={rising_time:.2f}s, "
              f"zeta={damping_ratio:.3f}")

except FileNotFoundError:
    print(f"Notice: {step_csv} not found. Skipping step response plots.")

print("\n--- Plotting complete ---")