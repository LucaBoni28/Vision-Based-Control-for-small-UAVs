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

from classes.config import CalibrationConfig, AppConfig
from classes.camera import CameraSource
from classes.detector import Detector, YoloDetector
from classes.video_streamer import VideoStreamer


# Defines the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic
class DistanceEstimator:
    # Initializes the DistanceEstimator with the given calibration configuration
    def __init__(self, calibration: CalibrationConfig, camera: CameraSource, detector: Detector, video_streamer: VideoStreamer = None):
        self._config = calibration
        self._camera = camera
        self._detector = detector
        self._video_streamer = video_streamer
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
            #"calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
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
        """
        Automated calibration mode:
        1. User places target at a known distance and confirms via keyboard
        2. System captures N frames (config.frames_per_sample), runs YOLO on each
        3. Averages the detected bounding box areas
        4. Saves (distance, avg_area) sample
        5. Repeat for multiple distances (minimum 2)
        6. Fit optical constant and save to calibration.json + update config.yaml
        """
        samples: List[Tuple[float, float]] = []  # (distance_m, avg_area_px)
        frames_per_sample = getattr(self._config, 'frames_per_sample', 30)

        print("=== Calibration Mode ===")
        print("Video stream will be sent to Ground Station for monitoring.")
        print(f"Place the target at a known distance, then press ENTER to capture {frames_per_sample} frames.")
        print("Press 'q' at the distance prompt to finish and fit.\n")

        # Check if we have a video streamer
        if self._video_streamer is None:
            print("WARNING: No VideoStreamer provided. Calibration frames will NOT be streamed to Ground Station.")
            print("         Video will only display locally (if display available).")

        while True:
            # Get distance from user
            try:
                dist_input = input("Enter the measured distance in meters (or 'q' to finish): ").strip()
                if dist_input.lower() == 'q':
                    break
                distance_m = float(dist_input)
                if distance_m <= 0:
                    print("Distance must be > 0, try again.")
                    continue
            except ValueError:
                print("Invalid number, try again.")
                continue

            print(f"Capturing {frames_per_sample} frames at distance {distance_m:.2f}m...")

            # Capture frames and collect areas
            areas = []
            frames_captured = 0

            while frames_captured < frames_per_sample:
                success, frame = self._camera.read()
                if not success:
                    print("Failed to read frame from camera")
                    break

                detections = self._detector.detect(frame)

                if detections:
                    # Use the largest detection
                    largest = max(detections, key=lambda d: (d.bbox[2] - d.bbox[0]) * (d.bbox[3] - d.bbox[1]))
                    x1, y1, x2, y2 = largest.bbox
                    area = (x2 - x1) * (y2 - y1)
                    areas.append(area)

                    # Draw visualization
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, f"area={area:.0f}px", (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    # No detection - still count the frame but don't add area
                    pass

                # Add overlay info
                cv2.putText(frame, f"Calibration: {distance_m:.2f}m  Frame {frames_captured+1}/{frames_per_sample}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(frame, f"Samples collected: {len(samples)}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # Stream to ground station if available
                if self._video_streamer is not None:
                    try:
                        self._video_streamer.send_frame(frame, 80)
                    except Exception as e:
                        print(f"Warning: Failed to stream frame: {e}")

                # # Also show locally if display available (for headless Jetson, this won't work)
                # try:
                #     cv2.imshow("Calibration", frame)
                #     cv2.waitKey(1)
                # except:
                #     pass  # No display available

                frames_captured += 1
                time.sleep(0.033)  # ~30 FPS

            if len(areas) < frames_per_sample * 0.5:  # Require at least 50% detection rate
                print(f"WARNING: Only {len(areas)}/{frames_per_sample} frames had detections. "
                      "Consider better lighting or target positioning.")

            if not areas:
                print("No detections captured at this distance. Skipping sample.")
                continue

            avg_area = sum(areas) / len(areas)
            samples.append((distance_m, avg_area))
            print(f"Sample #{len(samples)} captured: distance={distance_m:.2f}m, avg_area={avg_area:.0f}px "
                  f"(from {len(areas)} detections)\n")

        cv2.destroyAllWindows()

        if len(samples) < 2:
            print(f"\nOnly {len(samples)} sample(s) captured — need at least 2 to fit. Calibration NOT saved.")
            return

        # Fit optical constant
        optical_constant, rms_error_m = DistanceEstimator.fit(samples)
        print(f"\nFitted optical_constant = {optical_constant:.1f}")
        print(f"RMS distance error across the {len(samples)} calibration samples: {rms_error_m:.3f} m")

        if rms_error_m > 0.1:
            print("WARNING: RMS error is fairly high (>0.1 m). Consider re-capturing with more/cleaner samples "
                "before trusting this calibration for flight.")

        # Save to calibration.json
        self.save(optical_constant, samples, rms_error_m)
        print(f"Saved calibration to {self._config.file}")

        # Also update config.yaml with the new optical constant
        self._update_config_yaml(optical_constant)

    def _update_config_yaml(self, optical_constant: float) -> None:
        """Update the fallback_optical_constant in config.yaml with the calibrated value."""
        try:
            import yaml
            config_path = Path("classes/config.yaml")
            if not config_path.exists():
                config_path = Path("config.yaml")

            with open(config_path, "r") as f:
                config_data = yaml.safe_load(f)

            if "calibration" in config_data:
                config_data["calibration"]["fallback_optical_constant"] = optical_constant
                
                with open(config_path, "w") as f:
                    yaml.dump(config_data, f, default_flow_style=False, sort_keys=False)

                print(f"Updated config.yaml with optical_constant = {optical_constant:.1f}")
        except Exception as e:
            print(f"Warning: Could not update config.yaml: {e}")