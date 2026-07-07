###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the DistanceEstimator class, which encapsulates the bounding-box-area to distance calibration logic
###############################################################################

import json
import math
import time
import threading
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

    def calibrate(self) -> None:
        """
        Single-distance calibration mode with live preview:
        1. Video stream starts immediately for visual feedback
        2. User enters the known distance once
        3. User presses ENTER to start recording, ENTER again to stop
        4. Every frame with a detection produces an individual (distance, area) sample
        5. Optical constant K is fitted from all per-frame samples
        6. Results are saved to calibration.json and config.yaml
        """
        # Shared state between the preview thread and the main thread
        stop_preview = threading.Event()
        recording = threading.Event()
        areas: List[float] = []             # all per-frame area samples
        lock = threading.Lock()             # protects 'areas' and overlay text
        status_text = ["Waiting..."]        # mutable container for thread-safe overlay
        sample_count = [0]                  # live counter shown on overlay

        print("=== Calibration Mode ===")
        print("Live preview is starting... Stream is sent to the Ground Station.")

        if self._video_streamer is None:
            print("WARNING: No VideoStreamer provided. Calibration frames will NOT be streamed to Ground Station.")

        # ----- Background thread: capture → detect → draw → stream -----
        def preview_loop():
            while not stop_preview.is_set():
                success, frame = self._camera.read()
                if not success:
                    time.sleep(0.01)
                    continue

                detections = self._detector.detect(frame)

                if detections:
                    # Use the largest detection (by bounding box area)
                    largest = max(detections, key=lambda d: (d.bbox[2] - d.bbox[0]) * (d.bbox[3] - d.bbox[1]))
                    x1, y1, x2, y2 = largest.bbox
                    area = (x2 - x1) * (y2 - y1)

                    # If recording, accumulate the sample
                    if recording.is_set():
                        with lock:
                            areas.append(area)
                            sample_count[0] = len(areas)

                    # Draw bounding box and area value
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, f"area={area:.0f}px", (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                # Draw status overlay
                with lock:
                    overlay = status_text[0]
                    n_samples = sample_count[0]
                cv2.putText(frame, overlay, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(frame, f"Samples: {n_samples}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # Show recording indicator
                if recording.is_set():
                    cv2.circle(frame, (frame.shape[1] - 30, 30), 12, (0, 0, 255), -1)
                    cv2.putText(frame, "REC", (frame.shape[1] - 80, 38),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Stream to ground station if available
                if self._video_streamer is not None:
                    try:
                        self._video_streamer.send_frame(frame, 80)
                    except Exception as e:
                        print(f"Warning: Failed to stream frame: {e}")

                time.sleep(0.033)  # ~30 FPS

        # Start the preview thread immediately (video starts before any input)
        thread = threading.Thread(target=preview_loop, daemon=True)
        thread.start()

        # ----- Main thread: interact with the user via terminal -----
        try:
            # Step 1: get the known distance
            while True:
                try:
                    dist_input = input("\nEnter the known distance in meters: ").strip()
                    distance_m = float(dist_input)
                    if distance_m <= 0:
                        print("Distance must be > 0, try again.")
                        continue
                    break
                except ValueError:
                    print("Invalid number, try again.")

            with lock:
                status_text[0] = f"Distance: {distance_m:.2f}m — press ENTER to START recording"

            # Step 2: wait for ENTER to start recording
            input(f"\nPlace the target at {distance_m:.2f}m, then press ENTER to start recording...")
            with lock:
                status_text[0] = f"RECORDING at {distance_m:.2f}m — press ENTER to STOP"
            recording.set()
            print("Recording... keep the target steady. Press ENTER to stop.")

            # Step 3: wait for ENTER to stop recording
            input()
            recording.clear()

        finally:
            # Always stop the preview thread
            stop_preview.set()
            thread.join(timeout=3.0)

        # ----- Process results -----
        with lock:
            collected_areas = list(areas)

        if not collected_areas:
            print("\nNo detections were captured during recording. Calibration NOT saved.")
            return

        print(f"\nRecording complete: {len(collected_areas)} per-frame samples collected.")

        # Fit the optical constant using all per-frame samples
        optical_constant, rms_error_m = DistanceEstimator.fit_single_distance(distance_m, collected_areas)
        print(f"Fitted optical_constant = {optical_constant:.1f}")
        print(f"RMS distance error across {len(collected_areas)} samples: {rms_error_m:.3f} m")

        if rms_error_m > 0.1:
            print("WARNING: RMS error is fairly high (>0.1 m). Consider re-capturing with "
                  "better lighting or target positioning before trusting this calibration for flight.")

        # Save to calibration.json
        self.save(optical_constant, distance_m, collected_areas, rms_error_m)
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