###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the CameraSource abstraction and the CSICameraSource class,
#         which handles the Jetson Orin NX's CSI camera via a GStreamer pipeline
###############################################################################

from abc import ABC, abstractmethod     # Import abstract base class and abstract method decorators
import cv2                              # Import OpenCV for video capture and image processing
from config import CameraConfig         # Import CameraConfig for configuration YAML file

# Abstract base class for any camera source that can provide a stream of BGR frames to the MissionController
class CameraSource(ABC):

    # Abstract methods that must be implemented by subclasses
    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError

    # 
    @abstractmethod
    def read(self):
        raise NotImplementedError

    @abstractmethod
    def release(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def is_opened(self) -> bool:
        raise NotImplementedError

# Concrete implementation for the Jetson Orin NX's CSI camera
class CSICameraSource(CameraSource):

    # Initializes the CSICameraSource with the given camera configuration
    def __init__(self, camera_config: CameraConfig):
        self._config = camera_config
        self._cap = None

    # Opens the CSI camera using a GStreamer pipeline defined in the configuration
    def open(self) -> None:
        pipeline = self._config.gst_pipeline()
        print("Opening CSI Camera...")
        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError("OpenCV cannot open the hardware camera stream.")

    # Reads a frame from the CSI camera
    def read(self):
        return self._cap.read()

    # Releases the CSI camera
    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()

    # Checks if the CSI camera is opened
    def is_opened(self) -> bool:
        return self._cap is not None and self._cap.isOpened()
