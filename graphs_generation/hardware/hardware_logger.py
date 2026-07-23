###############################################################################
# Author: Luca Boninsegna
# Date:   25/06/26
# Descr:  Generation of CSV files for CPU/GPU usage with different tracking algorithms
###############################################################################


from jtop import jtop
import csv
import time
import sys

# Pass the algorithm name from the terminal to name the file
algorithm_name = sys.argv[1] if len(sys.argv) > 1 else "benchmark"
filename = f"{algorithm_name}_hardware_stats.csv"

print(f"Initializing Jetson Hardware Logger for: {algorithm_name}")
print("Press Ctrl+C to stop recording.")

# Initialize the jtop connection
with jtop() as jetson:
    if jetson.ok():
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Define the CSV headers
            writer.writerow(['Time_Sec', 'GPU_Util_%', 'GPU_Freq_MHz', 'CPU_Util_%', 'RAM_Usage_%', 'Power_TOT_mW'])
            
            start_time = time.time()
            
            try:
                while jetson.ok():
                    current_time = time.time() - start_time
                    
                    # Extract the metrics (jtop handles the low-level Tegra API calls)
                    # GPU: use jetson.gpu property for reliable TensorRT/CUDA utilization
                    gpu = 0
                    gpu_freq_mhz = 0
                    try:
                        gpu_info = jetson.gpu
                        for gpu_name, gpu_data in gpu_info.items():
                            if isinstance(gpu_data, dict):
                                gpu = gpu_data.get('load', gpu_data.get('val', 0))
                                freq = gpu_data.get('freq', {})
                                if isinstance(freq, dict):
                                    gpu_freq_mhz = freq.get('cur', 0) / 1000  # kHz -> MHz
                                elif isinstance(freq, (int, float)):
                                    gpu_freq_mhz = freq / 1000
                                break
                    except (AttributeError, TypeError, KeyError):
                        gpu = jetson.stats.get('GPU', 0)
                    
                    # Calculate average CPU usage across all cores
                    cpu_cores = [jetson.stats.get(f'CPU{i}', 0) for i in range(1, 9) if f'CPU{i}' in jetson.stats]
                    cpu_avg = sum(cpu_cores) / len(cpu_cores) if cpu_cores else 0
                    
                    ram = jetson.stats.get('RAM', 0)
                    power = jetson.stats.get('Power TOT', 0) # Total board power consumption
                    
                    # Log to CSV
                    writer.writerow([f"{current_time:.1f}", gpu, f"{gpu_freq_mhz:.0f}", f"{cpu_avg:.1f}", ram, power])
                    
                    # Force write to disk so data isn't lost if you abruptly kill the script
                    f.flush() 
                    time.sleep(1)
                    
            except KeyboardInterrupt:
                print(f"\nLogging complete. Data saved to {filename}")
    else:
        print("Error: jtop service is not running. Try 'sudo systemctl restart jtop.service'")