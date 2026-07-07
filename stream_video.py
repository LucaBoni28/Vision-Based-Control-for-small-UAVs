###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Receives the processed video stream from the Jetson Orin NX and
#         displays it on the Ground Station.
#
#         Clicking on the stream sends a normalized (x, y) point back to the
#         Jetson over the same TCP link, for ManualClickSelector to lock
#         onto whichever track the click landed inside.
###############################################################################

import cv2

# Import custom classes
from classes.config import AppConfig
from classes.video_streamer import StreamReceiver
from classes.click_command import CommandSender
from classes.distance_estimator import DistanceEstimator
from classes.camera import CSICameraSource


def main() -> None:
    # Load configuration YAML file
    config = AppConfig.load("classes/config.yaml")

    # Start the video stream receiver
    receiver = StreamReceiver(config.video_link)
    receiver.start()

    # Create a resizable window for displaying the video stream
    window_name = "Jetson Tracking Stream"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(
        window_name,
        config.display.ground_station_window_width,
        config.display.ground_station_window_height,
    )

    # Set up mouse callback for manual target selection
    if config.target_selection.mode == "manual":
        # Set up a command sender to send click coordinates back to the Jetson
        command_sender = CommandSender(config.command_link)
        # Get the window dimensions for normalization
        window_w = config.display.ground_station_window_width
        window_h = config.display.ground_station_window_height

        # Define the mouse callback function
        def on_mouse(event, x, y, flags, userdata):
            # Only handle left mouse button clicks
            if event == cv2.EVENT_LBUTTONDOWN:
                norm_x = x / window_w
                norm_y = y / window_h
                command_sender.send_click(norm_x, norm_y)
                print(f"Sent click at ({norm_x:.2f}, {norm_y:.2f}) to Jetson")
        # Set the mouse callback for the window
        cv2.setMouseCallback(window_name, on_mouse)
        print("Manual target selection: click the video window to select a target.")

    # Start the main loop to read and display frames
    while True:
        frame = receiver.read_frame()
        if frame is None:
            break

        cv2.putText(frame, "Press 'c' to calibrate", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        cv2.imshow(window_name, frame)

        if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) == -1:
            break
        elif cv2.waitKey(1) & 0xFF == ord('c'):
            print("Calibration requested. Press 'c' to stop calibration...")
            distance_estimator = DistanceEstimator(config, CSICameraSource(config.camera))


if __name__ == "__main__":
    main()
