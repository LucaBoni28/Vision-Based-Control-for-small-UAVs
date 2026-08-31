###############################################################################
# Author: Luca Boninsegna
# Date:   03/07/2026
# Descr:  This is the main entry point for the Jetson Orin NX mission.
#         It initializes all the necessary components:
#         - camera
#         - flight controller
#         - YOLO model
#         - tracker
#         - distance estimator
#         - target selector
#         - video streamer
###############################################################################

import sys

from ultralytics import YOLO

# Import all the classes needed for the mission
from classes.config import AppConfig
from classes.camera import CSICameraSource
from classes.flight_controller import FlightController
from classes.video_streamer import VideoStreamer, MjpegServer
from classes.tracker import create_tracker
from classes.distance_estimator import DistanceEstimator
from classes.detector import YoloDetector
from classes.target_selector import create_target_selector
from classes.click_command import CommandReceiver
from classes.mission_controller import MissionController



def main() -> None:
    # Load the configuration YAML file
    config = AppConfig.load("classes/config.yaml")

    # Initialize the flight controller and connect to the drone
    flight = FlightController(config.mavlink)
    flight.connect()


    # Initialize the YOLO model and tracker
    model = YOLO(config.model.path, task=config.model.task)
    tracker = create_tracker(config, model)
    print(f"Tracker backend: {config.tracker.backend}")
    detector = YoloDetector(model, config.detection)

    # Create the target selector based on the configuration
    target_selector = create_target_selector(config)
    print(f"Target selection mode: {config.target_selection.mode}")

    # Initialize the command receiver (used for clicks and remote calibration in any mode)
    command_receiver = CommandReceiver(config.command_link)
    print(f"Listening for Ground Station commands on UDP port {config.command_link.port}")

    # Initialize the video streamer (TCP, to ground station)
    streamer = VideoStreamer(config.video_link)
    streamer.connect()

    # Initialize the MJPEG HTTP server (for phone/tablet viewing)
    mjpeg_server = MjpegServer(config.mjpeg_server)
    mjpeg_server.start()

    # Initialize the distance estimator
    distance_estimator = DistanceEstimator(config.calibration)

    # Initialize the camera source
    camera = CSICameraSource(config.camera)
    try:
        camera.open()
    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Create the mission controller and run the mission
    try:
        mission = MissionController(
            config, camera, flight, streamer, tracker, target_selector,
            distance_estimator, detector, command_receiver,
            mjpeg_server=mjpeg_server,
        )
        mission.run()
    finally:
        command_receiver.close()
        mjpeg_server.stop()

# Run the main function if this script is executed directly
if __name__ == "__main__":
    main()
