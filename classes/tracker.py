###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the Tracker abstraction and its concrete implementations, which encapsulate the object tracking logic for the tracking system
###############################################################################

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

from classes.config import DetectionConfig, DeepSortConfig
from detector import YoloDetector


@dataclass
class Track:
    """One tracked object, backend-agnostic."""
    id: int
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2

# Abstract base class for any object tracker that can process a frame and return a list of tracks
class Tracker(ABC):
    @abstractmethod
    def track(self, frame) -> List[Track]:
        """Runs detection + tracking on one frame, returns the current tracks."""
        raise NotImplementedError

# Concrete implementation of the Tracker interface that uses Ultralytics' built-in tracking functionality,
# which combines detection and tracking in a single call to model.track()
class _UltralyticsNativeTracker(Tracker):
    # Initializes the _UltralyticsNativeTracker with the given YOLO model, detection configuration, and tracker YAML file
    def __init__(self, model, detection_config: DetectionConfig, tracker_yaml: str):
        self._model = model
        self._detection_config = detection_config
        self._tracker_yaml = tracker_yaml

    # Runs detection and tracking on the given frame, returning a list of Track objects representing the currently tracked objects
    def track(self, frame) -> List[Track]:
        results = self._model.track(
            frame,
            classes=self._detection_config.classes,
            conf=self._detection_config.confidence,
            persist=True,
            tracker=self._tracker_yaml,
            verbose=False,
        )

        if results[0].boxes.id is None:
            return []

        boxes = results[0].boxes.xyxy.cpu().numpy()
        ids = results[0].boxes.id.int().cpu().numpy()

        return [
            Track(id=int(track_id), bbox=tuple(box))
            for box, track_id in zip(boxes, ids)
        ]

# Concrete implementation of the Tracker interface that uses the ByteTrack algorithm for tracking supported by Ultralytics
class ByteTrackTracker(_UltralyticsNativeTracker):
    def __init__(self, model, detection_config: DetectionConfig, tracker_yaml: str = "bytetrack.yaml"):
        super().__init__(model, detection_config, tracker_yaml)

# Concrete implementation of the Tracker interface that uses the BotSort algorithm for tracking supported by Ultralytics
class BotSortTracker(_UltralyticsNativeTracker):
    def __init__(self, model, detection_config: DetectionConfig, tracker_yaml: str = "botsort.yaml"):
        super().__init__(model, detection_config, tracker_yaml)

# Concrete implementation of the Tracker interface that uses the DeepSort algorithm for tracking,
# which requires a separate detection pass and conversion of detections into the format expected by the DeepSort library
class DeepSortTracker(Tracker):
    # Initializes the DeepSortTracker with the given YOLO model, detection configuration, and DeepSort configuration
    def __init__(self, model, detection_config: DetectionConfig, deepsort_config: DeepSortConfig):
        try:
            from deep_sort_realtime.deepsort_tracker import DeepSort
        except ImportError as e:
            raise ImportError(
                "tracker.backend is 'deepsort' but the 'deep_sort_realtime' "
                "package isn't installed. Run: pip install deep-sort-realtime"
            ) from e

        self._detector = YoloDetector(model, detection_config)
        self._deepsort = DeepSort(
            max_age=deepsort_config.max_age,
            embedder=deepsort_config.embedder,
            half=deepsort_config.half,
            max_cosine_distance=deepsort_config.max_cosine_distance,
            n_init=deepsort_config.n_init,
            max_iou_distance=deepsort_config.max_iou_distance,
        )

    # Runs detection on the given frame, converts the detections into the format expected by DeepSort, and updates the DeepSort tracker to return a list of currently tracked objects as Track instances
    def track(self, frame) -> List[Track]:
        detections = self._detector.detect(frame)

        detections_for_tracker = [
            ([d.bbox[0], d.bbox[1], d.bbox[2] - d.bbox[0], d.bbox[3] - d.bbox[1]], d.confidence, d.class_id)
            for d in detections
        ]

        raw_tracks = self._deepsort.update_tracks(detections_for_tracker, frame=frame)

        tracks = []
        for t in raw_tracks:
            if not t.is_confirmed():
                continue
            if t.time_since_update > 2:
                continue
            x1, y1, x2, y2 = t.to_ltrb()
            tracks.append(Track(id=int(t.track_id), bbox=(x1, y1, x2, y2)))

        return tracks

# Factory function that creates the appropriate Tracker instance based on the configuration
def create_tracker(config, model) -> Tracker:
    """Factory: builds the right Tracker subclass from config.tracker.backend."""
    backend = config.tracker.backend

    if backend == "bytetrack":
        return ByteTrackTracker(model, config.detection, config.tracker.config_file)
    elif backend == "botsort":
        return BotSortTracker(model, config.detection, config.tracker.config_file)
    elif backend == "deepsort":
        return DeepSortTracker(model, config.detection, config.tracker.deepsort)
    else:
        raise ValueError(
            f"Unknown tracker backend '{backend}'. Expected: bytetrack, botsort, deepsort"
        )
