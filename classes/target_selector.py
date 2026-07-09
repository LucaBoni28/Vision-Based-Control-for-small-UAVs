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
# then locks onto the track whose bounding box contains the click position (x1 < click_x < x2 and y1 < click_y < y2).
# If multiple boxes contain the click, the one with the smallest area is chosen (most precise).
# A click outside all bounding boxes is ignored.
class ManualClickSelector(TargetSelector):
    # Initializes the ManualClickSelector with the given CommandReceiver, frame dimensions, and maximum click distance in pixels
    def __init__(
        self,
        command_receiver: CommandReceiver,
        frame_width: int,
        frame_height: int,
    ):
        self._command_receiver = command_receiver
        self._frame_width = frame_width
        self._frame_height = frame_height

    # Selects the track ID of the detected object whose bounding box contains the click position.
    # If multiple boxes contain the click, the one with the smallest area is chosen.
    # Returns None if no click or no box contains the click.
    def select(self, tracks: List[Track]) -> Optional[int]:
        if not tracks:
            return None

        click = self._command_receiver.poll_click()
        
        if click is None:
            return None

        norm_x, norm_y = click
        click_x = norm_x * self._frame_width
        click_y = norm_y * self._frame_height

        # Find all tracks whose bounding box contains the click
        containing_tracks = []
        for track in tracks:
            x1, y1, x2, y2 = track.bbox
            print(f"x1: {x1} | x2: {x2} | y1: {y1} | y2: {y2} ")
            if x1 <= click_x <= x2 and y1 <= click_y <= y2:
                area = (x2 - x1) * (y2 - y1)
                containing_tracks.append((track.id, area))

        if not containing_tracks:
            print(f"Click ignored: no bounding box contains the click at ({click_x:.0f}, {click_y:.0f})")
            return None

        # Choose the track with the largest area (closest object)
        best_id, best_area = max(containing_tracks, key=lambda t: t[1])
        print(f"Manual selection: locking onto track {best_id} (click inside bbox, area={best_area:.0f}px)")
        return best_id

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
        print("SELECTION MODE")
        return ManualClickSelector(
            command_receiver,
            frame_width=config.camera.width,
            frame_height=config.camera.height
        )
    else:
        raise ValueError(f"Unknown target_selection.mode '{mode}'. Expected: auto, manual, nearest")
