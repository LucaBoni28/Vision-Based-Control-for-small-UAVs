# Test 1: Inner Loop Verification

This folder contains the tools to tune and verify the inner velocity control loop of the drone in SITL. These tests completely bypass the vision-based PID outer loop and inject pure velocity commands directly to the drone via MAVLink.

The goal is to ensure the drone can accurately reach and maintain a commanded velocity (`vx`, `vy`, or `vz`) with minimal delay, overshoot, and steady-state error.

## Workflow 1: Automated Tuning

If the drone is oscillating or responding too slowly to velocity commands, you can run the automated tuner. This script sweeps through combinations of Proportional (P) and Integral (I) gains, runs a short step response for each, and scores them to find the best tune.

1. **Start the Tuning Process**
   Use `auto_tune_inner.py` to specify the axis and the range of gains to test.
   ```bash
   cd sitl_tests/test_1_inner_loop/
   
   # Example: Tune the forward velocity (vx) axis
   python auto_tune_inner.py --axis vx \
       --p-min 1.0 --p-max 3.0 --p-step 0.5 \
       --i-min 0.5 --i-max 1.5 --i-step 0.5
   ```

2. **Check the Output**
   - The tuner will print a table of all tested combinations sorted by score.
   - It saves a summary table CSV in `logs/` (e.g., `autotune_vx_summary_<timestamp>.csv`).
   - It saves the time-series CSV for the *best* tune only (e.g., `logs/autotune_vx_p2.00_i1.40.csv`).

3. **Plot the Best Tune**
   Use the plotting script on the best CSV generated:
   ```bash
   python plot_test_1.py --csv logs/autotune_vx_p2.00_i1.40.csv
   ```
   This will generate `plots/autotune_vx_p2.00_i1.40.png` and a PDF equivalent.

---

## Workflow 2: Single Step Response Verification

Once you have set the optimal gains (or if you just want to verify the current ArduPilot defaults), you can run a single step response test.

1. **Run the Step Test**
   Use `test_1_inner_loop.py` to inject a steady step velocity command.
   ```bash
   cd sitl_tests/test_1_inner_loop/
   
   # Example: Apply a 1.5 m/s step to the forward axis (vx) for 10 seconds
   python test_1_inner_loop.py --axis vx --velocity 1.5 --duration 10
   ```
   *Note: If testing `vz` (altitude), a negative velocity means climbing.*

2. **Check the Output**
   - The test data is logged to `logs/test_1_step_<axis>.csv`.

3. **Plot the Results**
   ```bash
   python plot_test_1.py --csv logs/test_1_step_vx.csv
   ```
   - This generates `plots/test_1_step_vx.png` (and PDF).
   - The script will also output a table of metrics to the terminal including Delay Time, Rise Time, Steady-State Error, and Overshoot.

---

## Folder Structure Reference
* `auto_tune_inner.py`: Grid-search tuner for velocity PID gains.
* `test_1_inner_loop.py`: Single step-response verification script.
* `plot_test_1.py`: Reads a CSV and outputs graphs & metrics.
* `logs/`: Where all `.csv` files are saved.
* `plots/`: Where all generated `.png` and `.pdf` charts are saved.

---

## Important Note: `yaw_rate` Axis Tuning
You may notice that the `yaw_rate` axis is not programmatically tuned using these scripts. This is due to architectural limitations in the current MAVLink bridging setup:

1. **GUIDED Mode Limitations:** ArduCopter's `SET_POSITION_TARGET_LOCAL_NED` command strictly accepts `vx, vy, vz` velocity vectors. It natively ignores the `yaw_rate` field when in GUIDED mode.
2. **ACRO Mode & RC Overrides:** To directly command the `ATC_RAT_YAW` PID loop, one must switch to ACRO mode and inject raw yaw stick inputs using `RC_CHANNELS_OVERRIDE`. However, the current SITL bridge architecture acts as a read-only telemetry link for some channels and drops/fails to forward programmatic RC overrides sent by our python script to the SITL drone.
3. **Conclusion:** Because the script cannot reliably inject yaw rate commands to measure the step response, it's impossible to use `auto_tune_inner.py` for yaw. Instead, we rely on **ArduCopter's default `ATC_RAT_YAW` gains**, which are highly optimized out-of-the-box for most standard multicopter frames.
