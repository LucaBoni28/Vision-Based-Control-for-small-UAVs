###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic.
#
#         Calibration is non-blocking and driven externally by MissionController:
#           1. start_recording(distance_m)  — enter calibration mode
#           2. record_sample(detections)    — called each frame while recording
#           3. stop_recording()             — fit, save, and return to idle
###############################################################################

import json
import math
from pathlib import Path
from typing import List, Tuple

from classes.config import CalibrationConfig
from classes.detector import Detection


# Defines the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic
class DistanceEstimator:
    # Initializes the DistanceEstimator with the given calibration configuration
    def __init__(self, calibration: CalibrationConfig):
        self._config = calibration
        self._optical_constant = None

        # Non-blocking calibration state
        self._recording = False
        self._recording_distance_m = 0.0
        self._recorded_areas: List[float] = []

        self.load()

    # Loads the calibration from the specified file, or uses the fallback optical constant if the file does not exist
    def load(self) -> None:
        path = Path(self._config.file)
        if path.exists():
            with open(path, "r") as f:
                data = json.load(f)
            self._optical_constant = data["optical_constant"]
            print(
                f"Loaded calibration from {path}: optical_constant={self._optical_constant:.1f} "
                f"(fit from {data.get('num_samples', '?')} samples, "
                f"RMS error {data.get('rms_error_m', float('nan')):.3f} m)"
            )
        else:
            self._optical_constant = self._config.optical_constant
            print(
                f"Calibration file '{path}' not found. "
                f"Using default optical_constant={self._optical_constant:.1f}. "
                f"Press 'c' on the Ground Station to calibrate."
            )


    # Saves the calibration to the specified file, including the optical constant, distance, areas, and RMS error
    def save(self, optical_constant: float, distance_m: float, areas: List[float], rms_error_m: float) -> None:
        path = Path(self._config.file)
        data = {
            "optical_constant": optical_constant,
            "distance_m": distance_m,
            "num_samples": len(areas),
            "areas_px": areas,
            "rms_error_m": rms_error_m,
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        self._optical_constant = optical_constant

    @property
    def optical_constant(self) -> float:
        return self._optical_constant

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def sample_count(self) -> int:
        return len(self._recorded_areas)

    # Returns the expected bounding box area in pixels for a given desired distance in meters, based on the optical constant
    def target_area(self, desired_distance_m: float) -> float:
        return self._optical_constant / (desired_distance_m ** 2)

    # Returns the estimated distance in meters for a given bounding box area in pixels, based on the optical constant
    def distance_to(self, bbox_area_px: float) -> float:
        if bbox_area_px <= 0:
            return 0.0
        return math.sqrt(self._optical_constant / bbox_area_px)

    # ── Non-blocking calibration methods (called by MissionController) ──────

    # Enters calibration recording mode at the given known distance
    def start_recording(self, distance_m: float) -> None:
        self._recording = True
        self._recording_distance_m = distance_m
        self._recorded_areas = []
        print(f"=== Calibration: RECORDING at {distance_m:.2f} m ===")

    # Records a single frame's target area during calibration.
    def record_sample(self, area: float) -> None:
        if not self._recording:
            return
        self._recorded_areas.append(float(area))

    # Stops recording, fits the optical constant, saves calibration, and returns to idle.
    # Returns True if calibration was successful, False if no samples were collected.
    def stop_recording(self) -> bool:
        self._recording = False

        if not self._recorded_areas:
            print("Calibration: no detections were captured during recording. NOT saved.")
            return False

        print(f"Calibration: {len(self._recorded_areas)} per-frame samples collected.")

        # Fit the optical constant using all per-frame samples
        optical_constant, rms_error_m = DistanceEstimator.fit_single_distance(
            self._recording_distance_m, self._recorded_areas
        )
        optical_constant = float(optical_constant)
        rms_error_m = float(rms_error_m)
        print(f"Fitted optical_constant = {optical_constant:.1f}")
        print(f"RMS distance error across {len(self._recorded_areas)} samples: {rms_error_m:.3f} m")

        if rms_error_m > 0.1:
            print("WARNING: RMS error is fairly high (>0.1 m). Consider re-capturing with "
                  "better lighting or target positioning before trusting this calibration for flight.")

        # Save to calibration.json
        self.save(optical_constant, self._recording_distance_m, self._recorded_areas, rms_error_m)
        print(f"Saved calibration to {self._config.file}")

        # Also update config.yaml with the new optical constant
        self._update_config_yaml(optical_constant)

        # Update the target area based on new calibration
        self._recorded_areas = []
        return True

    # ── Static fitting methods ──────────────────────────────────────────────

    # Performs a least-squares fit of the calibration data to determine the optical constant and RMS error, returning them as a tuple
    @staticmethod
    def fit(samples: List[Tuple[float, float]]) -> Tuple[float, float]:
        if len(samples) < 2:
            raise ValueError("Need at least 2 samples to fit a calibration.")

        xs = [1.0 / (d ** 2) for d, _ in samples]
        ys = [a for _, a in samples]

        numerator = sum(x * y for x, y in zip(xs, ys))
        denominator = sum(x * x for x in xs)
        if denominator == 0:
            raise ValueError("Cannot fit calibration: all distance samples are degenerate (distance=0?).")

        k = numerator / denominator

        squared_errors = []
        for d, a in samples:
            predicted_d = math.sqrt(k / a) if a > 0 else 0.0
            squared_errors.append((predicted_d - d) ** 2)
        rms_error_m = math.sqrt(sum(squared_errors) / len(squared_errors))

        return k, rms_error_m

    # Computes the optical constant from per-frame samples collected at a single known distance
    @staticmethod
    def fit_single_distance(distance_m: float, areas: List[float]) -> Tuple[float, float]:
        if not areas:
            raise ValueError("No area samples to fit.")

        # K = d^2 * mean(area)
        mean_area = sum(areas) / len(areas)
        k = (distance_m ** 2) * mean_area

        # RMS error: for each sample, compute the predicted distance and compare to the known distance
        squared_errors = []
        for a in areas:
            predicted_d = math.sqrt(k / a) if a > 0 else 0.0
            squared_errors.append((predicted_d - distance_m) ** 2)
        rms_error_m = math.sqrt(sum(squared_errors) / len(squared_errors))

        return k, rms_error_m

    def _update_config_yaml(self, optical_constant: float) -> None:
        """Update the optical_constant in config.yaml with the calibrated value."""
        try:
            import yaml
            config_path = Path("classes/config.yaml")
            if not config_path.exists():
                config_path = Path("config.yaml")

            with open(config_path, "r") as f:
                config_data = yaml.safe_load(f)

            if "calibration" in config_data:
                config_data["calibration"]["optical_constant"] = optical_constant
                
                with open(config_path, "w") as f:
                    yaml.dump(config_data, f, default_flow_style=False, sort_keys=False)

                print(f"Updated config.yaml with optical_constant = {optical_constant:.1f}")
        except Exception as e:
            print(f"Warning: Could not update config.yaml: {e}")