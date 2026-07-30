# Test 2: Outer Loop Step Response

This folder contains the tools to tune and verify the vision-based PID outer loop of the drone. These tests simulate the entire visual targeting pipeline, including the virtual camera projection, area estimation, and proportional-derivative (PD) controller logic.

The goal is to ensure the drone can accurately align with, approach, and track a visual target with minimal overshoot, fast rise time, and good settling stability.

## Tuning Workflow

The general workflow for tuning the outer loop is to isolate one axis at a time, apply a step input (instantly moving the target), and plot the drone's response. Based on the plot's metrics, you can adjust the corresponding gains in `classes/config.yaml`.

### 1. Run the Step Response Test
Use `test_2_step_response.py` to isolate an axis and record the drone's behavior as it attempts to follow the moving target.

```bash
cd sitl_tests/test_2_step_response/

# Example 1: Tune Forward Approach (Distance)
# Places the drone 0.6m from the target (the stopping distance), waits 5s for it to settle,
# then moves the target 3.0m away and records the approach.
python3 test_2_step_response.py --step-axis dist

# Example 2: Tune Yaw Tracking
# Places target 5.0m away, waits for drone to perfectly center it,
# then instantly shifts the target 3.0m to the right and records the rotation.
python3 test_2_step_response.py --step-axis yaw

# Example 3: Tune Altitude Tracking
# Places target 5.0m away, waits for drone to center it vertically,
# then instantly shifts the target 3.0m up and records the climb.
python3 test_2_step_response.py --step-axis alt
```

### 2. Plot and Analyze Results
Once the test finishes, a CSV log is saved in `logs/`. Use the plotting script to generate graphs and calculate control metrics (Rise Time, Overshoot, Settling Time).

```bash
# Plot the test you just ran
python3 plot_test_2.py --axis dist
# OR --axis yaw, --axis alt
```
*This will generate a summary PDF in `plots/` and print the metrics to your terminal.*

### 3. Adjust Gains in `config.yaml`
Open `classes/config.yaml` and locate the `control:` section. Modify the gains based on the plot you just generated:

- **If the response is too slow (high Rise Time):** Increase the Proportional Gain (`k_p`).
- **If the response overshoots heavily or oscillates:** Increase the Derivative Gain (`k_d`) to add damping, or slightly reduce `k_p`.
- **If the drone never fully reaches the target:** The deadzone might be too large, or you might need to adjust the `k_p` gain.

**Relevant Gains by Axis:**
- **dist (Forward Approach):** `k_p_vx` and `k_d_vx`
- **yaw (Horizontal Tracking):** `k_p_yaw` and `k_d_yaw`
- **alt (Vertical Tracking):** `k_p_vz` and `k_d_vz`

*Repeat steps 1-3 until you achieve a fast, smooth response with minimal overshoot!*

---

## Command Line Arguments Reference

**test_2_step_response.py**
- `--step-axis {yaw,alt,dist}`: (Required) Which axis to isolate and test.
- `--step-magnitude FLOAT`: How far to abruptly move the target (meters). Default: `3.0`
- `--initial-distance FLOAT`: Starting distance between drone and target (meters). Default: `0.6` for dist, `5.0` for yaw/alt.
- `--settle-before FLOAT`: Time (in seconds) to let the drone stabilize on the target before the step occurs. Default: `5.0`
- `--record-after FLOAT`: Time (in seconds) to record data after the step occurs. Default: `10.0`

**plot_test_2.py**
- `--axis {yaw,alt,dist}`: (Required) Which axis log to read and plot. Automatically looks for `logs/test_2_<axis>.csv`.
- `--csv PATH`: Optional override to directly specify a specific CSV file.
