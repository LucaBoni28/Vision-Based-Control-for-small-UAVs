# Test 3: Frequency Response (Bode Plots)

This folder contains the tools to evaluate the frequency response of the vision-based outer loop controller. By injecting sinusoidal oscillations into the target's position at various frequencies, we can measure how well the drone tracks a moving target. 

The output is used to generate **Bode plots** (Gain and Phase margins) which reveal the bandwidth of the system—telling you exactly how fast a target can move before the drone fails to keep up.

## Workflow

The workflow consists of running a frequency sweep (which automatically tests multiple frequencies in sequence) and then plotting the results.

### 1. Run the Frequency Sweep
Use `test_3_frequency_response.py` to command the target to oscillate sinusoidally and record the drone's response. The script automatically sweeps through a logarithmic range of frequencies.

```bash
cd sitl_tests/test_3_frequency_response/

# Example 1: Test Forward Tracking (Distance)
# The target will oscillate forward and backward. 
# By default, it tests frequencies from 0.05 Hz to 2.0 Hz.
python test_3_frequency_response.py --axis distance

# Example 2: Test Horizontal Tracking (Yaw)
# The target will oscillate left and right across the drone's field of view.
python test_3_frequency_response.py --axis yaw

# Example 3: Test Vertical Tracking (Altitude)
# The target will oscillate up and down.
python test_3_frequency_response.py --axis altitude
```

*Note: The script automatically groups logs into numbered folders (e.g., `logs/distance/run_001/`) to keep experiments organized.*

### 2. Plot the Bode Response
Once the frequency sweep is complete, use `plot_test_3.py` to analyze the generated CSVs. The script extracts the input and output signals, fits sinusoidal curves to them, and computes the Amplitude Ratio (Gain) and Phase Shift.

```bash
cd sitl_tests/test_3_frequency_response/

# Plot the most recent distance run
python plot_test_3.py --axis distance

# Plot a specific older run (e.g., run_002) for yaw
python plot_test_3.py --axis yaw --run-name run_002
```

This script will:
1. Print a summary table of the Gain (dB) and Phase (°) for each frequency to your terminal.
2. Generate a comprehensive Bode Plot (`test_3_bode_distance.png` / `.pdf`) in the corresponding `plots/<axis>/<run_name>/` folder.
3. Generate individual time-domain plots for *every single frequency tested*, so you can visually verify the sine wave fitting.
4. Save the raw Gain/Phase tabular data to `test_3_bode_results_<axis>.csv`.

---

## Command Line Arguments Reference

### `test_3_frequency_response.py`
- `--axis {yaw,altitude,distance}`: Which axis to oscillate. (Default: `yaw`)
- `--amplitude FLOAT`: Amplitude of the oscillation in meters (or degrees for yaw). (Default: `3.0`)
- `--initial-distance FLOAT`: Starting distance between drone and target. (Default: `10.0`)
- `--frequencies STRING`: Comma-separated list of frequencies to test. (Default: Logarithmic sweep `0.05, 0.1, 0.2, 0.3, ..., 2.0`)
- `--duration-per-freq FLOAT`: How long to record data at each frequency (seconds). (Default: `30.0`)
- `--settle-time FLOAT`: Wait time before starting the oscillation to let the drone stabilize. (Default: `10.0`)
- `--single-freq FLOAT`: Run only one specific frequency instead of a full sweep.
- `--run-name STRING`: Name of the subfolder to save logs in. (Default: `auto`, which auto-increments `run_001`, `run_002`, etc.)

### `plot_test_3.py`
- `--axis {yaw,altitude,distance}`: Which axis to analyze.
- `--run-name STRING`: Which run folder to analyze. (Default: `auto`, which picks the highest numbered run folder).
- `--output-dir PATH`: Optional override to save plots to a specific custom directory.
