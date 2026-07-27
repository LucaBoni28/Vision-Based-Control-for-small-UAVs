###############################################################################
# Author: Luca Boninsegna
# Date:   23/07/26
# Descr:  Comprehensive visualization of combined telemetry & hardware variables
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

log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')

# Define algorithms
algorithms = ['bytetrack', 'botsort', 'deepsort']
colors = ['darkred', 'darkblue', 'darkgreen']
pretty_names = {'bytetrack': 'ByteTrack', 'botsort': 'BoT-SORT', 'deepsort': 'DeepSORT'}

# =============================================================================
# 1. HARDWARE STATISTICS PLOTTING (ALGORITHM COMPARISON)
# =============================================================================
fig_hw, axs_hw = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

hw_found = False
for algo_name, color in zip(algorithms, colors):
    filename = os.path.join(log_dir, f'{algo_name}_hardware_stats.csv')
    try:
        df_hw = pd.read_csv(filename) 
        if 'GPU_Util_%' in df_hw.columns:
            axs_hw[0].plot(df_hw['Time_Sec'], df_hw['GPU_Util_%'], color=color, linewidth=1.5, alpha=0.8, label=pretty_names[algo_name])
        axs_hw[1].plot(df_hw['Time_Sec'], df_hw['CPU_Util_%'], color=color, linewidth=1.5, alpha=0.8, label=pretty_names[algo_name])
        axs_hw[2].plot(df_hw['Time_Sec'], df_hw['RAM_Usage_%'], color=color, linewidth=1.5, alpha=0.8, label=pretty_names[algo_name])
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
    plt.savefig('hardware_comparison_stats.png', format='png', dpi=300)
    print("Hardware stats comparison plot successfully generated.")

# =============================================================================
# 2. COMBINED TRACKING BENCHMARK ANALYSIS (FPS, Latency & ID Switches)
# =============================================================================
avg_fps = {}
id_switches = {}
avg_latency = {}
avg_jitter = {}
trackers_found = []

for algo_name in algorithms:
    filepath = os.path.join(log_dir, f'combined_{algo_name}.csv')
    try:
        df_bench = pd.read_csv(filepath)
        p_name = pretty_names[algo_name]
        trackers_found.append(p_name)

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

if trackers_found:
    fig_bench, axs_bench = plt.subplots(1, 4, figsize=(20, 5)) # 4 subplots now
    colors_bench = ['#2ecc71', '#3498db', '#e74c3c']
    x_pos = range(len(trackers_found))

    # FPS Bar Chart
    fps_values = [avg_fps[t] for t in trackers_found]
    bars = axs_bench[0].bar(x_pos, fps_values, color=colors_bench[:len(trackers_found)], width=0.5)
    axs_bench[0].set_xticks(list(x_pos))
    axs_bench[0].set_xticklabels(trackers_found)
    axs_bench[0].set_ylabel('Average FPS')
    axs_bench[0].set_title('Algorithm FPS Comparison')
    axs_bench[0].grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars, fps_values):
        axs_bench[0].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
                     f'{val:.1f}', ha='center', va='bottom', fontweight='bold')
                     
    # Latency Bar Chart
    lat_values = [avg_latency[t] for t in trackers_found]
    bars_lat = axs_bench[1].bar(x_pos, lat_values, color=colors_bench[:len(trackers_found)], width=0.5)
    axs_bench[1].set_xticks(list(x_pos))
    axs_bench[1].set_xticklabels(trackers_found)
    axs_bench[1].set_ylabel('Pipeline Latency [ms]')
    axs_bench[1].set_title('Pipeline Latency Comparison')
    axs_bench[1].grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars_lat, lat_values):
        axs_bench[1].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
                     f'{val:.1f}', ha='center', va='bottom', fontweight='bold')

    # ID Switch Bar Chart
    switch_values = [id_switches[t] for t in trackers_found]
    bars2 = axs_bench[2].bar(x_pos, switch_values, color=colors_bench[:len(trackers_found)], width=0.5)
    axs_bench[2].set_xticks(list(x_pos))
    axs_bench[2].set_xticklabels(trackers_found)
    axs_bench[2].set_ylabel('ID Switches')
    axs_bench[2].set_title('Tracking ID Switch Comparison')
    axs_bench[2].grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars2, switch_values):
        axs_bench[2].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.1,
                     f'{val}', ha='center', va='bottom', fontweight='bold')

    # Jitter Bar Chart
    jitter_values = [avg_jitter[t] for t in trackers_found]
    bars_jitter = axs_bench[3].bar(x_pos, jitter_values, color=colors_bench[:len(trackers_found)], width=0.5)
    axs_bench[3].set_xticks(list(x_pos))
    axs_bench[3].set_xticklabels(trackers_found)
    axs_bench[3].set_ylabel('Average Jitter [pixels]')
    axs_bench[3].set_title('Bounding Box Jitter')
    axs_bench[3].grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars_jitter, jitter_values):
        axs_bench[3].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
                     f'{val:.1f}', ha='center', va='bottom', fontweight='bold')

    plt.figure(fig_bench.number)
    plt.tight_layout()
    plt.savefig('benchmark_comparison.png', format='png', dpi=300)
    print("Benchmark & Latency comparison plot saved as benchmark_comparison.png")

# =============================================================================
# 3. INDIVIDUAL TELEMETRY PLOTS (Velocities, Distances, Areas per algorithm)
# =============================================================================
for algo_name in algorithms:
    filepath = os.path.join(log_dir, f'combined_{algo_name}.csv')
    try:
        df_dist = pd.read_csv(filepath)
        # Filter only when we have a valid target (Distance > 0)
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
        axs_dist[1].plot(df_dist['Time_Sec'], df_dist['omega_z'], color='orange', linewidth=1.5, label='Yaw Rate ($\omega_z$)')
        axs_dist[1].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8)
        axs_dist[1].set_ylabel('Ang. Vel [rad/s]')
        axs_dist[1].grid(True, linestyle=':', alpha=0.7)
        axs_dist[1].legend(loc='upper right')

        # 3. Distance Estimation
        axs_dist[2].plot(df_dist['Time_Sec'], df_dist['Distance_Est'], color='red', linewidth=1.5, label='Estimated Distance')
        axs_dist[2].axhline(0.70, color='green', linestyle='--', linewidth=2, label='Target Hover Threshold')
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
        plt.savefig(f'telemetry_{algo_name}.png', format='png', dpi=300)
        print(f"Telemetry plot saved for {p_name}.")
        
    except FileNotFoundError:
        pass

# =============================================================================
# 4. TEST 2: STEP RESPONSE ANALYSIS (Overshoot, Settling Time, Damping Ratio)
# =============================================================================
try:
    step_csv = os.path.join(log_dir, 'step_response.csv')
    df_step = pd.read_csv(step_csv)

    fig_step, axs_step = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Top subplot: Error vs Time
    axs_step[0].plot(df_step['Time_Sec'], df_step['e_x'], color='blue', linewidth=1.5, label='$e_x$ (Horizontal Error)')
    axs_step[0].axhline(0, color='black', linestyle='-', linewidth=0.5)

    # Settling band (+/- 2% of step size)
    step_size = df_step['e_x'].iloc[0]  # First value = step magnitude
    settling_band = 0.02 * abs(step_size)
    axs_step[0].axhspan(-settling_band, settling_band, alpha=0.15, color='green', label=f'$\pm$2% Settling Band')

    # Overshoot
    min_ex = df_step['e_x'].min()
    if min_ex < 0:
        overshoot_pct = abs(min_ex) / abs(step_size) * 100
        overshoot_idx = df_step['e_x'].idxmin()
        axs_step[0].annotate(f'Overshoot: {overshoot_pct:.1f}%',
                         xy=(df_step['Time_Sec'].iloc[overshoot_idx], min_ex),
                         xytext=(df_step['Time_Sec'].iloc[overshoot_idx] + 0.5, min_ex - 0.05),
                         arrowprops=dict(arrowstyle='->', color='red'),
                         color='red', fontsize=10)
    else:
        overshoot_pct = 0.0

    # Settling time
    settled = df_step['e_x'].abs() <= settling_band
    settling_time = df_step['Time_Sec'].iloc[-1]
    for i in range(len(settled) - 1, -1, -1):
        if not settled.iloc[i]:
            if i + 1 < len(settled):
                settling_time = df_step['Time_Sec'].iloc[i + 1]
            break
    axs_step[0].axvline(settling_time, color='orange', linestyle='--', linewidth=1.5, label=f'Settling Time: {settling_time:.2f}s')

    # RMSE
    rmse = np.sqrt(np.mean(df_step['e_x'] ** 2))

    # Damping ratio from overshoot percentage
    if overshoot_pct > 0:
        ln_os = np.log(overshoot_pct / 100)
        damping_ratio = -ln_os / np.sqrt(np.pi**2 + ln_os**2)
    else:
        damping_ratio = 1.0  # Critically damped or over-damped

    # Metrics text box
    textstr = (f'Overshoot: {overshoot_pct:.1f}%\n'
               f'Settling Time: {settling_time:.2f}s\n'
               f'RMSE: {rmse:.4f}\n'
               f'Damping Ratio $\zeta$: {damping_ratio:.3f}')
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    axs_step[0].text(0.98, 0.98, textstr, transform=axs_step[0].transAxes, fontsize=10,
                 verticalalignment='top', horizontalalignment='right', bbox=props)

    axs_step[0].set_ylabel('Normalized Error $e_x$')
    axs_step[0].set_title('Yaw PD Controller Step Response')
    axs_step[0].grid(True, linestyle=':', alpha=0.7)
    axs_step[0].legend(loc='upper left')

    # Bottom subplot: Control Effort
    axs_step[1].plot(df_step['Time_Sec'], df_step['omega_z'], color='orange', linewidth=1.5, label='$\omega_z$ (Yaw Rate)')
    axs_step[1].axhline(0, color='black', linestyle='-', linewidth=0.5)
    axs_step[1].set_xlabel('Time [s]')
    axs_step[1].set_ylabel('Yaw Rate [rad/s]')
    axs_step[1].set_title('Control Effort')
    axs_step[1].grid(True, linestyle=':', alpha=0.7)
    axs_step[1].legend(loc='upper right')

    plt.figure(fig_step.number)
    plt.tight_layout()
    plt.savefig('step_response_analysis.png', format='png', dpi=300)
    print(f"Step response plot saved. RMSE={rmse:.4f}, Overshoot={overshoot_pct:.1f}%, "
          f"Settling={settling_time:.2f}s, zeta={damping_ratio:.3f}")

except FileNotFoundError:
    print("Notice: logs/step_response.csv not found. Skipping step response plot.")

# Show all generated plots simultaneously on the screen
# plt.show()