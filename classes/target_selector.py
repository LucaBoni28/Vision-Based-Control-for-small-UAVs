###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the TargetSelector abstraction and its concrete implementations,
#         which determine which detected object to lock onto based on the configured selection mode
###############################################################################

from abc import ABC, abstractmethod
from typing import List, Optional

from classes.tracker import Track
from classes.click_command import CommandReceiver

# Abstract base class for any target selector that can choose a track to lock onto from a list of detected tracks
class TargetSelector(ABC):
    # Abstract method that must be implemented by subclasses to select a track ID from a list of tracks
    @abstractmethod
    def select(self, tracks: List[Track]) -> Optional[int]:
        raise NotImplementedError

# Concrete implementation of TargetSelector that locks onto the first detected track
class FirstDetectedSelector(TargetSelector):
    # Selects the first detected track from the list of tracks, or None if no tracks are present
    def select(self, tracks: List[Track]) -> Optional[int]:
        return tracks[0].id if tracks else None

# Concrete implementation of TargetSelector that waits for an operator click relayed from the ground station via CommandReceiver,
# then locks onto whichever track's bounding-box center is closest to that click, as long as it's within max_click_distance_px.
# A click that lands nowhere near any detected object is ignored rather than locking the nearest thing regardless of distance.
class ManualClickSelector(TargetSelector):
    # Initializes the ManualClickSelector with the given CommandReceiver, frame dimensions, and maximum click distance in pixels
    def __init__(
        self,
        command_receiver: CommandReceiver,
        frame_width: int,
        frame_height: int,
        max_click_distance_px: float,
    ):
        self._command_receiver = command_receiver
        self._frame_width = frame_width
        self._frame_height = frame_height
        self._max_click_distance_px = max_click_distance_px

    # Selects the track ID of the detected object whose bounding-box center is closest to the operator's click, if within max_click_distance_px; otherwise returns None
    def select(self, tracks: List[Track]) -> Optional[int]:
        if not tracks:
            return None

        click = self._command_receiver.poll_click()
        if click is None:
            return None

        norm_x, norm_y = click
        click_x = norm_x * self._frame_width
        click_y = norm_y * self._frame_height

        best_id = None
        best_dist = None
        for track in tracks:
            x1, y1, x2, y2 = track.bbox
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            dist = ((center_x - click_x) ** 2 + (center_y - click_y) ** 2) ** 0.5
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_id = track.id

        if best_dist is not None and best_dist <= self._max_click_distance_px:
            print(f"Manual selection: locking onto track {best_id} ({best_dist:.0f}px from click)")
            return best_id

        print(f"Click ignored: nearest object was {best_dist:.0f}px away "
              f"(max {self._max_click_distance_px}px)")
        return None

# Concrete implementation of TargetSelector that locks onto the detected object with the largest bounding-box area, which is assumed to be the closest to the drone
class NearestObjectSelector(TargetSelector):
    # Selects the track ID of the detected object with the largest bounding-box area, or None if no tracks are present
    def select(self, tracks: List[Track]) -> Optional[int]:
        if not tracks:
            return None
        
        # Returns the ID of the track with the largest bounding-box area, which is assumed to be the closest object to the drone
        def area(track: Track) -> float:
            x1, y1, x2, y2 = track.bbox
            return (x2 - x1) * (y2 - y1)

        return max(tracks, key=area).id

# Factory function that creates the appropriate TargetSelector instance based on the configuration and optional CommandReceiver
def create_target_selector(config, command_receiver: Optional[CommandReceiver] = None) -> TargetSelector:
    mode = config.target_selection.mode

    if mode == "auto":
        return FirstDetectedSelector()
    elif mode == "nearest":
        return NearestObjectSelector()
    elif mode == "manual":
        if command_receiver is None:
            raise ValueError("target_selection.mode is 'manual' but no CommandReceiver was provided")
        return ManualClickSelector(
            command_receiver,
            frame_width=config.camera.width,
            frame_height=config.camera.height,
            max_click_distance_px=config.target_selection.max_click_distance_px,
        )
    else:
        raise ValueError(f"Unknown target_selection.mode '{mode}'. Expected: auto, manual, nearest")
