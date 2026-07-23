###############################################################################
# Author: Luca Boninsegna
# Date:   25/06/26
# Descr:  Comprehensive visualization of all generated telemetry variables
#         (v_x, v_z, omega_z, Distance_Est, BB_Area) sharing a time axis.
###############################################################################

import pandas as pd
import matplotlib.pyplot as plt

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

# Create a new figure for the hardware stats (2 rows, sharing the X-axis)
fig2, axs2 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# Loop through each algorithm's CSV file and plot its data
for (algo_name, filename), color in zip(hardware_files.items(), colors):
    try:
        df_hw = pd.read_csv(filename) 
        
        # Plot CPU Usage on the upper subplot
        axs2[0].plot(df_hw['Time_Sec'], df_hw['CPU_Util_%'], color=color, linewidth=1.5, alpha=0.8, label=f'{algo_name}')
        
        # Plot RAM Usage on the lower subplot
        axs2[1].plot(df_hw['Time_Sec'], df_hw['RAM_Usage_%'], color=color, linewidth=1.5, alpha=0.8, label=f'{algo_name}')
        
    except FileNotFoundError:
        print(f"Notice: {filename} not found. Skipping {algo_name}.")

# Format the Upper Plot (CPU)
axs2[0].set_ylabel('CPU Util [%]')
axs2[0].set_title('Hardware Utilization Comparison Across Tracking Algorithms')
axs2[0].grid(True, linestyle=':', alpha=0.7)
axs2[0].legend(loc='upper right', fontsize=10)

# Format the Lower Plot (RAM)
axs2[1].set_xlabel('Time [s]')
axs2[1].set_ylabel('RAM Usage [%]')
axs2[1].grid(True, linestyle=':', alpha=0.7)
axs2[1].legend(loc='upper right', fontsize=10)

# Export the comparative Hardware Stats plot
plt.tight_layout()
plt.savefig('hardware_comparison_stats.png', format='png', dpi=300)
print("Hardware stats comparison plot successfully generated and saved as PNG.")

# Show all generated plots simultaneously on the screen
plt.show()