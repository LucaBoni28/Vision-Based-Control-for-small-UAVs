###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the Detector abstraction and the YoloDetector implementation,
#         which encapsulate the object detection logic for the tracking system
###############################################################################

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

from config import DetectionConfig


@dataclass
class Detection:
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2 — clamped to frame bounds
    confidence: float
    class_id: int

# Abstract base class for any object detector that can process a frame and return a list of detections
class Detector(ABC):
    @abstractmethod
    def detect(self, frame) -> List[Detection]:
        raise NotImplementedError

# Concrete implementation of the Detector interface using the Ultralytics YOLO model
class YoloDetector(Detector):
    # Initializes the YoloDetector with the given YOLO model, detection configuration, and minimum bounding box size in pixels
    def __init__(self, model, detection_config: DetectionConfig, min_box_size_px: int = 20):
        self._model = model
        self._detection_config = detection_config
        self._min_box_size_px = min_box_size_px

    # Performs object detection on the given frame and returns a list of Detection objects
    def detect(self, frame) -> List[Detection]:
        img_h, img_w = frame.shape[:2]

        # Run the YOLO model on the frame with the specified classes and confidence threshold, without verbose output
        results = self._model.predict(
            frame,
            classes=self._detection_config.classes,
            conf=self._detection_config.confidence,
            verbose=False,
        )

        # Process the results to extract bounding boxes, confidence scores, and class IDs, filtering out small boxes at the edges of the frame
        detections = []
        for box in results[0].boxes:
            raw_x1, raw_y1, raw_x2, raw_y2 = box.xyxy[0].cpu().numpy()

            x1 = max(0, int(raw_x1))
            y1 = max(0, int(raw_y1))
            x2 = min(img_w, int(raw_x2))
            y2 = min(img_h, int(raw_y2))

            if (x2 - x1) < self._min_box_size_px or (y2 - y1) < self._min_box_size_px:
                continue  # filter degenerate boxes at the screen edges

            detections.append(Detection(
                bbox=(x1, y1, x2, y2),
                confidence=box.conf[0].item(),
                class_id=int(box.cls[0].item()),
            ))
        return detections
