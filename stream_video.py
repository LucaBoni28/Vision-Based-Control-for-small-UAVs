###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Receives the processed video stream from the Jetson Orin NX and
#         displays it on the Ground Station.
#
#         Keyboard commands:
#           - Click on the stream: sends a normalized (x, y) point to the
#             Jetson for ManualClickSelector to lock onto a target.
#           - Press 'c': toggles calibration mode. First press prompts for
#             the known distance and sends CALIBRATE_START. Second press
#             sends CALIBRATE_STOP.
#           - Press 'q': quit.
###############################################################################

import cv2

# Import custom classes
from classes.config import AppConfig
from classes.video_streamer import StreamReceiver
from classes.click_command import CommandSender


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

    # Always create a command sender (used for clicks + calibration commands)
    command_sender = CommandSender(config.command_link)

    # OpenCV's mouse callback already reports (x, y) in image pixel
    # coordinates (it remaps internally for WINDOW_NORMAL), so we normalize directly by the stream image dimensions.
    stream_w = config.display.stream_width
    stream_h = config.display.stream_height

    # Set up mouse callback for manual target selection
    def on_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            norm_x = x / stream_w
            norm_y = y / stream_h
            norm_x = max(0.0, min(1.0, norm_x))
            norm_y = max(0.0, min(1.0, norm_y))
            command_sender.send_click(norm_x, norm_y)
            print(f"Sent click at ({norm_x:.2f}, {norm_y:.2f}) to Jetson")

    cv2.setMouseCallback(window_name, on_mouse)

    if config.target_selection.mode == "manual":
        print("Manual target selection: click the video window to select a target.")

    # Calibration state machine
    calibrating = False

    # Start the main loop to read and display frames
    while True:
        frame = receiver.read_frame()
        if frame is None:
            break

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) == -1:
            break

        elif key == ord('c'):
            if not calibrating:
                # Check with Jetson first if calibration is allowed
                if not command_sender.send_calibrate_check():
                    print("Calibration check failed (rejected by Jetson).")
                    continue

                # Prompt for distance on the PC terminal
                try:
                    dist_input = input("\n[Calibration] Enter the known distance in meters: ").strip()
                    distance_m = float(dist_input)
                    if distance_m <= 0:
                        print("Distance must be > 0. Calibration cancelled.")
                        continue
                except ValueError:
                    print("Invalid number. Calibration cancelled.")
                    continue

                # Send command and check confirmation response from Jetson
                if command_sender.send_calibrate_start(distance_m):
                    # Once distance is entered and confirmed, print instructions for what to do next with the live stream active
                    print("\n=== Remote Calibration Started ===")
                    print("1. Place the target object in front of the camera at the set distance.")
                    if config.target_selection.mode == "manual":
                        print("2. Lock onto the target by clicking on it in the video window.")
                    print(f"Recording samples at {distance_m:.2f}m. Press 'c' again in the video window to stop.")
                    calibrating = True
                else:
                    print("Calibration failed to start (rejected by Jetson).")
            else:
                command_sender.send_calibrate_stop()
                calibrating = False
                print("Sent CALIBRATE_STOP to Jetson.")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
