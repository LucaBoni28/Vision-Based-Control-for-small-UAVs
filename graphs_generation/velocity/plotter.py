###############################################################################
# Author: Luca Boninsegna
# Date:   25/06/26
# Descr:  Comprehensive visualization of all generated telemetry variables
#         (v_x, v_z, omega_z, Distance_Est, BB_Area) sharing a time axis.
###############################################################################

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 1. Load the validated flight data
df = pd.read_csv('thesis_distance_log.csv')

# 2. Configure academic formatting for LaTeX/PDF integration
plt.rcParams.update({
    'font.family': 'serif', 
    'font.size': 12
})

# Create a 4-row stacked figure. sharex=True forces them to align perfectly in time.
fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# 3. Plot 1: Translational Velocities (m/s)
axs[0].plot(df['Time_Sec'], df['v_x'], color='blue', linewidth=1.5, label='Forward Vel ($v_x$)')
axs[0].plot(df['Time_Sec'], df['v_z'], color='purple', linewidth=1.5, label='Vertical Vel ($v_z$)')
axs[0].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8) 
axs[0].set_ylabel('Velocity [m/s]')
axs[0].set_title('UAV Autonomous Approach Telemetry Data')
axs[0].grid(True, linestyle=':', alpha=0.7)
axs[0].legend(loc='upper right')

# 4. Plot 2: Rotational Velocity (rad/s)
axs[1].plot(df['Time_Sec'], df['omega_z'], color='orange', linewidth=1.5, label='Yaw Rate ($\omega_z$)')
axs[1].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8) 
axs[1].set_ylabel('Ang. Vel [rad/s]')
axs[1].grid(True, linestyle=':', alpha=0.7)
axs[1].legend(loc='upper right')

# 5. Plot 3: Target Distance Estimation (m)
axs[2].plot(df['Time_Sec'], df['Distance_Est'], color='red', linewidth=1.5, label='Estimated Distance')
axs[2].axhline(0.60, color='green', linestyle='--', linewidth=2, label='Target Hover Threshold')
axs[2].set_ylabel('Distance [m]')
axs[2].grid(True, linestyle=':', alpha=0.7)
axs[2].legend(loc='upper right')

# 6. Plot 4: Raw Vision Data (Pixels^2)
axs[3].plot(df['Time_Sec'], df['BB_Area'], color='teal', linewidth=1.5, label='Bounding Box Area')
axs[3].set_xlabel('Time [s]')
axs[3].set_ylabel('Area [px$^2$]')
axs[3].grid(True, linestyle=':', alpha=0.7)
axs[3].legend(loc='lower right')

# 7. Export for Document Insertion
plt.tight_layout()
plt.savefig('velo_commands_plots.png', format='png', dpi=300)
print("Comprehensive plot successfully generated and saved as PNG.")

# =============================================================================
# 8. HARDWARE STATISTICS PLOTTING (ALGORITHM COMPARISON)
# =============================================================================
# Define your files and their corresponding algorithm names for the legend
hardware_files = {
    'bytetrack': 'bytetrack_hardware_stats.csv',
    'botsort': 'botsort_hardware_stats.csv',
    'deepsort': 'deepsort_hardware_stats.csv'
}

# Colors for the different algorithms to keep the graphs distinct and readable
colors = ['darkred', 'darkblue', 'darkgreen']

# Create a new figure for the hardware stats (3 rows: GPU, CPU, RAM)
fig2, axs2 = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

# Loop through each algorithm's CSV file and plot its data
for (algo_name, filename), color in zip(hardware_files.items(), colors):
    try:
        df_hw = pd.read_csv(filename) 
        
        # Plot GPU Usage on the top subplot (now fixed to capture TensorRT loads)
        if 'GPU_Util_%' in df_hw.columns:
            axs2[0].plot(df_hw['Time_Sec'], df_hw['GPU_Util_%'], color=color, linewidth=1.5, alpha=0.8, label=f'{algo_name}')
        
        # Plot CPU Usage on the middle subplot
        axs2[1].plot(df_hw['Time_Sec'], df_hw['CPU_Util_%'], color=color, linewidth=1.5, alpha=0.8, label=f'{algo_name}')
        
        # Plot RAM Usage on the bottom subplot
        axs2[2].plot(df_hw['Time_Sec'], df_hw['RAM_Usage_%'], color=color, linewidth=1.5, alpha=0.8, label=f'{algo_name}')
        
    except FileNotFoundError:
        print(f"Notice: {filename} not found. Skipping {algo_name}.")

# Format the Top Plot (GPU)
axs2[0].set_ylabel('GPU Util [%]')
axs2[0].set_title('Hardware Utilization Comparison Across Tracking Algorithms')
axs2[0].grid(True, linestyle=':', alpha=0.7)
axs2[0].legend(loc='upper right', fontsize=10)

# Format the Middle Plot (CPU)
axs2[1].set_ylabel('CPU Util [%]')
axs2[1].grid(True, linestyle=':', alpha=0.7)
axs2[1].legend(loc='upper right', fontsize=10)

# Format the Bottom Plot (RAM)
axs2[2].set_xlabel('Time [s]')
axs2[2].set_ylabel('RAM Usage [%]')
axs2[2].grid(True, linestyle=':', alpha=0.7)
axs2[2].legend(loc='upper right', fontsize=10)

# Export the comparative Hardware Stats plot
plt.tight_layout()
plt.savefig('hardware_comparison_stats.png', format='png', dpi=300)
print("Hardware stats comparison plot successfully generated and saved as PNG.")

# =============================================================================
# 9. TEST 1: TRACKING BENCHMARK ANALYSIS (FPS & ID Switches)
# =============================================================================
log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs')
benchmark_csv_files = {
    'ByteTrack': os.path.join(log_dir, 'benchmark_bytetrack.csv'),
    'BoT-SORT':  os.path.join(log_dir, 'benchmark_botsort.csv'),
    'DeepSORT':  os.path.join(log_dir, 'benchmark_deepsort.csv'),
}

avg_fps = {}
id_switches = {}
trackers_found = []

for algo_name, filepath in benchmark_csv_files.items():
    try:
        df_bench = pd.read_csv(filepath)
        trackers_found.append(algo_name)

        # Average FPS from processing time
        avg_proc_ms = df_bench['Processing_Time_ms'].mean()
        avg_fps[algo_name] = 1000.0 / avg_proc_ms if avg_proc_ms > 0 else 0

        # Count ID switches: number of times Object_ID changes (excluding -1 gaps)
        valid_ids = df_bench[df_bench['Object_ID'] != -1]['Object_ID']
        switches = (valid_ids != valid_ids.shift()).sum() - 1  # first entry is not a switch
        id_switches[algo_name] = max(0, int(switches))

    except FileNotFoundError:
        print(f"Notice: {filepath} not found. Skipping {algo_name}.")

if trackers_found:
    fig3, axs3 = plt.subplots(1, 2, figsize=(12, 5))
    colors_bench = ['#2ecc71', '#3498db', '#e74c3c']
    x_pos = range(len(trackers_found))

    # FPS Bar Chart
    fps_values = [avg_fps[t] for t in trackers_found]
    bars = axs3[0].bar(x_pos, fps_values, color=colors_bench[:len(trackers_found)], width=0.5)
    axs3[0].set_xticks(list(x_pos))
    axs3[0].set_xticklabels(trackers_found)
    axs3[0].set_ylabel('Average FPS')
    axs3[0].set_title('Tracking Algorithm FPS Comparison')
    axs3[0].grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars, fps_values):
        axs3[0].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
                     f'{val:.1f}', ha='center', va='bottom', fontweight='bold')

    # ID Switch Bar Chart
    switch_values = [id_switches[t] for t in trackers_found]
    bars2 = axs3[1].bar(x_pos, switch_values, color=colors_bench[:len(trackers_found)], width=0.5)
    axs3[1].set_xticks(list(x_pos))
    axs3[1].set_xticklabels(trackers_found)
    axs3[1].set_ylabel('ID Switches')
    axs3[1].set_title('Tracking ID Switch Comparison')
    axs3[1].grid(True, axis='y', linestyle=':', alpha=0.7)
    for bar, val in zip(bars2, switch_values):
        axs3[1].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.1,
                     f'{val}', ha='center', va='bottom', fontweight='bold')

    plt.tight_layout()
    plt.savefig('benchmark_comparison.png', format='png', dpi=300)
    print("Benchmark comparison plot saved as benchmark_comparison.png")
else:
    print("Notice: No benchmark CSVs found. Skipping benchmark plot.")

# =============================================================================
# 10. TEST 2: STEP RESPONSE ANALYSIS (Overshoot, Settling Time, Damping Ratio)
# =============================================================================
try:
    step_csv = os.path.join(log_dir, 'step_response.csv')
    df_step = pd.read_csv(step_csv)

    fig4, axs4 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Top subplot: Error vs Time
    axs4[0].plot(df_step['Time_Sec'], df_step['e_x'], color='blue', linewidth=1.5, label='$e_x$ (Horizontal Error)')
    axs4[0].axhline(0, color='black', linestyle='-', linewidth=0.5)

    # Settling band (+/- 2% of step size)
    step_size = df_step['e_x'].iloc[0]  # First value = step magnitude
    settling_band = 0.02 * abs(step_size)
    axs4[0].axhspan(-settling_band, settling_band, alpha=0.15, color='green', label=f'$\pm$2% Settling Band')

    # Overshoot
    min_ex = df_step['e_x'].min()
    if min_ex < 0:
        overshoot_pct = abs(min_ex) / abs(step_size) * 100
        overshoot_idx = df_step['e_x'].idxmin()
        axs4[0].annotate(f'Overshoot: {overshoot_pct:.1f}%',
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
    axs4[0].axvline(settling_time, color='orange', linestyle='--', linewidth=1.5, label=f'Settling Time: {settling_time:.2f}s')

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
    axs4[0].text(0.98, 0.98, textstr, transform=axs4[0].transAxes, fontsize=10,
                 verticalalignment='top', horizontalalignment='right', bbox=props)

    axs4[0].set_ylabel('Normalized Error $e_x$')
    axs4[0].set_title('Yaw PD Controller Step Response')
    axs4[0].grid(True, linestyle=':', alpha=0.7)
    axs4[0].legend(loc='upper left')

    # Bottom subplot: Control Effort
    axs4[1].plot(df_step['Time_Sec'], df_step['omega_z'], color='orange', linewidth=1.5, label='$\omega_z$ (Yaw Rate)')
    axs4[1].axhline(0, color='black', linestyle='-', linewidth=0.5)
    axs4[1].set_xlabel('Time [s]')
    axs4[1].set_ylabel('Yaw Rate [rad/s]')
    axs4[1].set_title('Control Effort')
    axs4[1].grid(True, linestyle=':', alpha=0.7)
    axs4[1].legend(loc='upper right')

    plt.tight_layout()
    plt.savefig('step_response_analysis.png', format='png', dpi=300)
    print(f"Step response plot saved. RMSE={rmse:.4f}, Overshoot={overshoot_pct:.1f}%, "
          f"Settling={settling_time:.2f}s, zeta={damping_ratio:.3f}")

except FileNotFoundError:
    print("Notice: logs/step_response.csv not found. Skipping step response plot.")

# =============================================================================
# 11. TEST 3: DISTANCE & VELOCITY ENHANCED (+ Pitch + Pipeline Latency)
# =============================================================================
try:
    dist_csv = os.path.join(log_dir, 'distance.csv')
    df_dist = pd.read_csv(dist_csv)

    fig5, axs5 = plt.subplots(6, 1, figsize=(10, 16), sharex=True)

    # 1. Translational Velocities
    axs5[0].plot(df_dist['Time_Sec'], df_dist['v_x'], color='blue', linewidth=1.5, label='Forward Vel ($v_x$)')
    axs5[0].plot(df_dist['Time_Sec'], df_dist['v_z'], color='purple', linewidth=1.5, label='Vertical Vel ($v_z$)')
    axs5[0].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8)
    axs5[0].set_ylabel('Velocity [m/s]')
    axs5[0].set_title('UAV Autonomous Approach \u2014 Enhanced Telemetry')
    axs5[0].grid(True, linestyle=':', alpha=0.7)
    axs5[0].legend(loc='upper right')

    # 2. Yaw Rate
    axs5[1].plot(df_dist['Time_Sec'], df_dist['omega_z'], color='orange', linewidth=1.5, label='Yaw Rate ($\omega_z$)')
    axs5[1].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8)
    axs5[1].set_ylabel('Ang. Vel [rad/s]')
    axs5[1].grid(True, linestyle=':', alpha=0.7)
    axs5[1].legend(loc='upper right')

    # 3. Distance Estimation
    axs5[2].plot(df_dist['Time_Sec'], df_dist['Distance_Est'], color='red', linewidth=1.5, label='Estimated Distance')
    axs5[2].axhline(0.60, color='green', linestyle='--', linewidth=2, label='Target Hover Threshold')
    axs5[2].set_ylabel('Distance [m]')
    axs5[2].grid(True, linestyle=':', alpha=0.7)
    axs5[2].legend(loc='upper right')

    # 4. Bounding Box Area
    axs5[3].plot(df_dist['Time_Sec'], df_dist['A_real'], color='teal', linewidth=1.5, label='Bounding Box Area')
    axs5[3].set_ylabel('Area [px$^2$]')
    axs5[3].grid(True, linestyle=':', alpha=0.7)
    axs5[3].legend(loc='lower right')

    # 5. Pitch Angle (NEW)
    pitch_deg = df_dist['current_pitch_rad'] * 180.0 / np.pi
    axs5[4].plot(df_dist['Time_Sec'], pitch_deg, color='brown', linewidth=1.5, label='Pitch Angle')
    axs5[4].axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.8)
    axs5[4].set_ylabel('Pitch [deg]')
    axs5[4].grid(True, linestyle=':', alpha=0.7)
    axs5[4].legend(loc='upper right')

    # 6. Pipeline Latency (NEW)
    axs5[5].plot(df_dist['Time_Sec'], df_dist['Pipeline_Latency_ms'], color='gray', linewidth=1.0, alpha=0.6, label='Per-Frame Latency')
    avg_latency = df_dist['Pipeline_Latency_ms'].mean()
    axs5[5].axhline(avg_latency, color='red', linestyle='--', linewidth=2, label=f'Average: {avg_latency:.1f} ms')
    axs5[5].set_xlabel('Time [s]')
    axs5[5].set_ylabel('Latency [ms]')
    axs5[5].grid(True, linestyle=':', alpha=0.7)
    axs5[5].legend(loc='upper right')

    plt.tight_layout()
    plt.savefig('enhanced_distance_telemetry.png', format='png', dpi=300)
    print(f"Enhanced distance plot saved. Average pipeline latency: {avg_latency:.1f} ms")

except FileNotFoundError:
    print("Notice: logs/distance.csv not found. Skipping enhanced distance plot.")

# Show all generated plots simultaneously on the screen
plt.show()