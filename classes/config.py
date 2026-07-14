###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the AppConfig class, which loads and validates the configuration from a YAML file
###############################################################################

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import yaml

@dataclass
class CameraConfig:
    sensor_id: int
    width: int
    height: int
    framerate: int

    def gst_pipeline(self) -> str:
        """Builds the GStreamer pipeline string from these parameters."""
        return (
            f"nvarguscamerasrc sensor-id={self.sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={self.width}, height={self.height}, "
            f"framerate={self.framerate}/1, format=NV12 ! "
            "nvvidconv ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink"
        )

@dataclass
class MavlinkConfig:
    connection: str
    telemetry_output: str
    source_system: int
    source_component: int
    attitude_stream_rate_hz: int


@dataclass
class VideoLinkConfig:
    host: str
    port: int

@dataclass
class ModelConfig:
    path: str
    task: str


@dataclass
class DetectionConfig:
    classes: List[int]
    confidence: float


@dataclass
class DeepSortConfig:
    max_age: int
    embedder: str
    half: bool
    max_cosine_distance: float
    n_init: int
    max_iou_distance: float


@dataclass
class TrackerConfig:
    backend: str
    config_file: str
    max_timeout_frames: int
    deepsort: "DeepSortConfig"


@dataclass
class ControlConfig:
    k_p_yaw: float
    k_d_yaw: float
    k_p_vz: float
    k_d_vz: float
    k_p_vx: float
    k_d_vx: float
    r_stop: float
    yaw_deadzone: float
    vz_deadzone: float
    area_deadzone: float
    max_derivative_dt: float


@dataclass
class CalibrationConfig:
    file: str
    desired_stopping_distance_m: float
    optical_constant: float
    frames_per_sample: int = 30  # Number of frames to capture and average per distance sample


@dataclass
class DisplayConfig:
    stream_width: int
    stream_height: int
    jpeg_quality: int
    ground_station_window_width: int
    ground_station_window_height: int


@dataclass
class TargetSelectionConfig:
    mode: str

@dataclass
class CommandLinkConfig:
    jetson_host: str
    port: int

@dataclass
class PitchCompensationConfig:
    mode: str

# Defines the AppConfig class, which loads and validates the configuration from a YAML file
@dataclass
class AppConfig:
    mavlink: MavlinkConfig
    video_link: VideoLinkConfig
    camera: CameraConfig
    model: ModelConfig
    detection: DetectionConfig
    tracker: TrackerConfig
    control: ControlConfig
    calibration: CalibrationConfig
    display: DisplayConfig
    target_selection: TargetSelectionConfig
    command_link: CommandLinkConfig
    pitch_compensation: PitchCompensationConfig
    # Loads the configuration from a YAML file at the given path and returns an AppConfig instance
    @staticmethod
    def load(path: str | Path = "config.yaml") -> "AppConfig":
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(
                f"Config file not found at '{path}'. "
                "Copy config.yaml next to your script, or pass the correct path."
            )

        with open(path, "r") as f:
            raw = yaml.safe_load(f)

        try:
            config = AppConfig(
                mavlink=MavlinkConfig(**raw["mavlink"]),
                video_link=VideoLinkConfig(**raw["video_link"]),
                camera=CameraConfig(**raw["camera"]),
                model=ModelConfig(**raw["model"]),
                detection=DetectionConfig(**raw["detection"]),
                tracker=TrackerConfig(
                    backend=raw["tracker"]["backend"],
                    config_file=raw["tracker"]["config_file"],
                    max_timeout_frames=raw["tracker"]["max_timeout_frames"],
                    deepsort=DeepSortConfig(**raw["tracker"]["deepsort"]),
                ),
                control=ControlConfig(**raw["control"]),
                calibration=CalibrationConfig(**raw["calibration"]),
                display=DisplayConfig(**raw["display"]),
                target_selection=TargetSelectionConfig(**raw["target_selection"]),
                command_link=CommandLinkConfig(**raw["command_link"]),
                pitch_compensation=PitchCompensationConfig(**raw.get("pitch_compensation", {"mode": "software"})),
            )
        except KeyError as e:
            raise ValueError(f"config.yaml is missing required section/key: {e}") from e
        except TypeError as e:
            raise ValueError(
                f"config.yaml has a mismatched or missing field: {e}"
            ) from e

        config._validate()
        return config

    # Performs basic sanity checks on the configuration to ensure that values are within expected ranges and constraints
    def _validate(self) -> None:
        """Basic sanity checks so bad config fails fast, at startup, not mid-flight."""
        errors = []

        if not (0.0 < self.detection.confidence <= 1.0):
            errors.append(
                f"detection.confidence must be in (0, 1], got {self.detection.confidence}"
            )
        if not (0 < self.video_link.port < 65536):
            errors.append(f"video_link.port out of range: {self.video_link.port}")
        if self.tracker.backend not in ("bytetrack", "botsort", "deepsort"):
            errors.append(
                f"tracker.backend must be one of bytetrack/botsort/deepsort, "
                f"got '{self.tracker.backend}'"
            )
        if self.control.r_stop <= 0:
            errors.append("control.r_stop must be > 0")
        if self.calibration.optical_constant <= 0:
            errors.append("calibration.optical_constant must be > 0")
        if self.calibration.desired_stopping_distance_m <= 0:
            errors.append("calibration.desired_stopping_distance_m must be > 0")
        if self.tracker.backend == "deepsort":
            if self.tracker.deepsort.max_age <= 0:
                errors.append("tracker.deepsort.max_age must be > 0")
            if self.tracker.deepsort.n_init <= 0:
                errors.append("tracker.deepsort.n_init must be > 0")
        if self.target_selection.mode not in ("auto", "manual", "nearest"):
            errors.append(
                f"target_selection.mode must be 'auto', 'manual', or 'nearest', got '{self.target_selection.mode}'"
            )
        if not (0 < self.command_link.port < 65536):
            errors.append(f"command_link.port out of range: {self.command_link.port}")
        valid_modes = ("software", "gimbal_auto", "gimbal_manual", "none")
        if self.pitch_compensation.mode not in valid_modes:
            errors.append(f"pitch_compensation.mode must be one of {valid_modes}, got '{self.pitch_compensation.mode}'")

        if errors:
            raise ValueError(
                "Invalid config.yaml:\n  - " + "\n  - ".join(errors)
            )


if __name__ == "__main__":
    # Load the configuration from config.yaml and print some key values for verification
    cfg = AppConfig.load("config.yaml")
    print("Config loaded OK.")
    print(f"  MAVLink: {cfg.mavlink.connection}")
    print(f"  Video link: {cfg.video_link.host}:{cfg.video_link.port}")
    print(f"  Tracker backend: {cfg.tracker.backend}")
    print(f"  Target area @ {cfg.calibration.desired_stopping_distance_m} m: "
          f"{cfg.calibration.target_area():.1f}")
