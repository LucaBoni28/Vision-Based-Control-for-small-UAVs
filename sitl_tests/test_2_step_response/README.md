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
python test_2_step_response.py --step-axis dist

# Example 2: Tune Yaw Tracking
# Places target 5.0m away, waits for drone to perfectly center it,
# then instantly shifts the target 3.0m to the right and records the rotation.
python test_2_step_response.py --step-axis yaw

# Example 3: Tune Altitude Tracking
# Places target 5.0m away, waits for drone to center it vertically,
# then instantly shifts the target 3.0m up and records the climb.
python test_2_step_response.py --step-axis alt
```

### 2. Plot and Analyze Results
Once the test finishes, a CSV log is saved in `logs/`. Use the plotting script to generate graphs and calculate control metrics (Rise Time, Overshoot, Settling Time).

```bash
# Plot the test you just ran
python plot_test_2.py --axis dist
# OR --axis yaw, --axis alt
```
*This will generate a summary PDF in `plots/` and print the metrics to your terminal.*

### 3. Adjust Gains in `config.yaml`
Open `classes/config.yaml` and locate the `control:` section. Modify the gains based on the plot you just generated:

- **If the response is too slow (high Rise Time):** Increase the Proportional Gain (`k_p`).
- **If the response overshoots heavily or oscillates:** Increase the Derivative Gain (`k_d`) to add damping, or slightly reduce `k_p`.
  > **⚠️ Vision Noise Caveat:** Be extremely careful with `k_d` when the error signal comes directly from raw bounding box pixel coordinates. Vision detection inherently has frame-to-frame jitter. Because the controller computes the derivative as a raw finite difference (`de/dt`), applying a `k_d` gain will heavily amplify this high-frequency noise into erratic, "spiky" motor commands. For axes where slight overshoot is acceptable (like **yaw**), a well-tuned **P-only controller (`k_d = 0`)** provides a much smoother and hardware-friendly response.
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

---

## Automated Tuning (`auto_tune_outer.py`)

If you don't want to guess-and-check gains manually, you can use the automated tuner. It runs a grid search over a range of P and D gains, modifies the configuration in-memory, executes the step response, and scores the result based on Settling Time and Overshoot.

### 1. Run the Autotuner
```bash
```bash
# Example 1: Tune distance (forward velocity) over a grid of P and D gains
python auto_tune_outer.py --axis dist --p-min 0.5 --p-max 1.5 --p-step 0.5 --d-min 0.02 --d-max 0.10 --d-step 0.02

# Example 2: P-only sweep for yaw (forcing D=0 to avoid noise amplification)
python auto_tune_outer.py --axis yaw --p-min 0.75 --p-max 0.95 --p-step 0.025 --d-min 0 --d-max 0 --d-step 1
```

The script will:
- Reuse a single flight session (so you don't have to wait for takeoff each time).
- Automatically recover altitude if it drops during aggressive altitude tests.
- Rank the combinations and save a summary CSV (e.g., `autotune_outer_dist_summary_12345.csv`).
- Only save the step response CSV file for the **absolute best** combination to save disk space.

### 2. Plot the Best Result
The autotuner will output the file path of the winning CSV log. You can plot it exactly like a manual test by using the `--csv` flag instead of `--axis`:

```bash
python plot_test_2.py --csv logs/autotune_outer_dist_p0.50_d0.0600_12345.csv
```

**Relevant Autotune Arguments:**
- `--axis {yaw,alt,dist}`: (Required) Which axis to tune.
- `--p-min`, `--p-max`, `--p-step`: Range for Proportional gain.
- `--d-min`, `--d-max`, `--d-step`: Range for Derivative gain.
- `--step-magnitude`, `--initial-distance`: (Optional) Defaults map intelligently just like the manual test script.

**plot_test_2.py**
- `--axis {yaw,alt,dist}`: (Required) Which axis log to read and plot. Automatically looks for `logs/test_2_<axis>.csv`.
- `--csv PATH`: Optional override to directly specify a specific CSV file.
