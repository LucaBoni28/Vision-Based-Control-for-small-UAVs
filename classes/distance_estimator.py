###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic
###############################################################################

import json
import math
import time
import cv2
from ultralytics import YOLO
from pathlib import Path
from typing import List, Tuple

from classes.config import CalibrationConfig
from classes.camera import CameraSource
from classes.detector import Detector, YoloDetector
from classes.config import AppConfig


# Defines the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic
class DistanceEstimator:
    # Initializes the DistanceEstimator with the given calibration configuration
    def __init__(self, calibration: CalibrationConfig, camera: CameraSource, detector: Detector):
        self._config = calibration
        self._camera = camera
        self._detector = detector
        self._optical_constant
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
            print(f"Calibration file '{path}' not found. Starting calibration...")
            self.calibrate()


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

    def calibrate(self) -> None:
    
        samples: List[Tuple[float, float]] = []  # (distance_m, area_px)

        print("=== Calibration Mode ===")
        print("Place the target at a known distance, press 'c' to capture, 'q' to finish and fit.\n")

        window_name = "Calibration"

        while True:
            
            success, frame = self._camera.read()
            if not success:
                break

            detections = self._detector.detect(frame)
            largest = None
            if detections:
                largest = max(detections, key=lambda d: (d.bbox[2] - d.bbox[0]) * (d.bbox[3] - d.bbox[1]))
                x1, y1, x2, y2 = largest.bbox
                area = (x2 - x1) * (y2 - y1)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"area={area:.0f}px", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                area = None

            cv2.putText(frame, f"Samples: {len(samples)}  |  'c' capture, 'q' finish",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                if area is None:
                    print("No target detected in frame — can't capture. Adjust position and try again.")
                    continue
                try:
                    dist = float(input("Enter the measured distance in meters for this sample: "))
                except ValueError:
                    print("Invalid number, skipping this sample.")
                    continue
                if dist <= 0:
                    print("Distance must be > 0, skipping this sample.")
                    continue
                samples.append((dist, area))
                print(f"Captured sample #{len(samples)}: distance={dist}m, area={area:.0f}px\n")
            elif key == ord('q'):
                break           

        cv2.destroyWindows(window_name)

        if len(samples) < 2:
            print(f"\nOnly {len(samples)} sample(s) captured — need at least 2 to fit. Calibration NOT saved.")
            return

        optical_constant, rms_error_m = DistanceEstimator.fit(samples)
        print(f"\nFitted optical_constant = {optical_constant:.1f}")
        print(f"RMS distance error across the {len(samples)} calibration samples: {rms_error_m:.3f} m")

        if rms_error_m > 0.1:
            print("WARNING: RMS error is fairly high (>0.1 m). Consider re-capturing with more/cleaner samples "
                "before trusting this calibration for flight.")

        self.save(optical_constant, samples, rms_error_m)
        print(f"Saved to {self._config.file}")