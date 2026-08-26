# Test 4: Trajectory Scenario Simulation

This test evaluates the full 3-axis tracking performance of the PD controller (identical to the one used in `mission_controller.py`) while the virtual target follows a predefined walking trajectory. 

Unlike previous tests (which isolated individual control axes), this test captures real-world coupling dynamics (e.g., yaw-induced altitude drift, forward velocity affecting centering) and allows for the injection of **realistic vision noise**.

## Components
- `test_4_trajectory.py`: Main execution script. Connects to SITL and closes the feedback loop.
- `trajectories.py`: Defines 5 multi-axis walking scenarios (straight, L-shape, circle, stop-and-go, approach/retreat).
- `plot_test_4.py`: Post-processing script to generate top-down maps, altitude profiles, error plots, and comparison metrics.
- `../utils/noisy_camera.py`: A wrapper around the `VirtualCamera` that models real-world YOLO imperfections (bounding box jitter, latency, dropouts, ID switches).

## Prerequisites

1. Ensure ArduPilot SITL and MAVProxy are running.
2. The drone should be on the ground in Guided mode, ready to arm.

## How to Run the Tests

Execute the scripts from the **project root directory**

### 1. Run a Single Scenario (Ideal Conditions)
Run a specific trajectory with perfect virtual vision:
```powershell
python sitl_tests/test_4_trajectory/test_4_trajectory.py --scenario straight_walk
```

### 2. Compare Ideal vs. Noisy Vision (Recommended)
This runs the selected scenario twice back-to-back: once with perfect vision, and once injecting YOLO bounding box jitter, latency, and dropouts based on estimated real-world characteristics. This is the best way to see exactly how vision noise degrades tracking smoothness.
```powershell
python sitl_tests/test_4_trajectory/test_4_trajectory.py --scenario circle --compare
```

### 3. Run All Scenarios
To run all predefined trajectories in a single batch:
```powershell
python sitl_tests/test_4_trajectory/test_4_trajectory.py --scenario all --compare
```

## Available Scenarios (`--scenario`)

We intentionally designed a mix of 2D and 3D scenarios to test different aspects of the controller.

**Pure 2D Scenarios (Constant Altitude):**
- `straight_walk`: Target walks 15m straight ahead at a constant 1.0 m/s, then stops. Altitude stays perfectly constant (`dz = 0.0`). Tests basic forward velocity control and stopping behavior.
- `circle`: Target walks in a continuous 5m radius circle. Altitude stays perfectly constant (`dz = 0.0`). Tests continuous yaw/forward coupling and lateral tracking.

**3D Scenarios (Variable Altitude):**
- `l_shape`: Target walks 8m forward, turns 90 degrees right, and climbs **2 meters** while walking sideways. Forces the drone to track laterally and vertically at the same time.
- `stop_and_go`: Target walks 5m, pauses, and repeats, gradually climbing **1.5 meters** in a staircase pattern (climbing only while walking). Tests responsiveness to sudden stops and starts while managing altitude.
- `approach_retreat`: Target walks down a ramp (descends **0.8m**) as it approaches the drone, pauses, then turns around and walks up a hill (climbs **1.8m**). This forces extreme coupling between forward velocity (`vx`) and vertical velocity (`vz`).

## Generating Plots

After running the simulation, generate the visualization plots by running `plot_test_4.py`. By default, the script will automatically find the most recent run in the `logs/` directory.

```powershell
python sitl_tests/test_4_trajectory/plot_test_4.py
```

To plot a specific previous run, use the `--run` argument matching the generated run folder:
```powershell
python sitl_tests/test_4_trajectory/plot_test_4.py --run run_002_circle_compare
```

**Output Structure:**
Data and plots are heavily organized. When a test is run, a log folder is created named with the pattern `run_<number>_<scenario>_<mode>`. The plotting script will automatically create a matching subfolder in the `plots/` directory (e.g., `plots/run_002_circle_compare/`) containing all generated graphs, keeping your results perfectly synchronized.
