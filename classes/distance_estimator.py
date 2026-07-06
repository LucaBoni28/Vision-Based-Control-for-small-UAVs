###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic
###############################################################################

import json
import math
import time
from pathlib import Path
from typing import List, Tuple

from classes.config import CalibrationConfig

# Defines the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic
class DistanceEstimator:
    # Initializes the DistanceEstimator with the given calibration configuration
    def __init__(self, calibration_config: CalibrationConfig):
        self._config = calibration_config
        self._optical_constant = None
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
            self._optical_constant = self._config.fallback_optical_constant
            print(
                f"WARNING: no calibration file found at '{path}'. Using "
                f"fallback_optical_constant={self._optical_constant:.1f} from config.yaml. "
                f"Run calibrate.py to generate a real calibration for this rig."
            )

    # Saves the calibration to the specified file, including the optical constant, number of samples, RMS error, and timestamp
    def save(self, optical_constant: float, samples: List[Tuple[float, float]], rms_error_m: float) -> None:
        path = Path(self._config.file)
        data = {
            "optical_constant": optical_constant,
            "num_samples": len(samples),
            "samples": [{"distance_m": d, "area_px": a} for d, a in samples],
            "rms_error_m": rms_error_m,
            "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        self._optical_constant = optical_constant

    @property
    def optical_constant(self) -> float:
        return self._optical_constant

    # Returns the expected bounding box area in pixels for a given desired distance in meters, based on the optical constant
    def target_area(self, desired_distance_m: float) -> float:
        return self._optical_constant / (desired_distance_m ** 2)

    # Returns the estimated distance in meters for a given bounding box area in pixels, based on the optical constant
    def distance_to(self, bbox_area_px: float) -> float:
        if bbox_area_px <= 0:
            return 0.0
        return math.sqrt(self._optical_constant / bbox_area_px)

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
