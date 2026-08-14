###############################################################################
# Author: Luca Boninsegna
# Date:   23/07/26
# Descr:  Lightweight CSV logger for thesis data collection.
#         Keeps all logging logic separate from the main pipeline (tracking.py).
#         Usage: logger = ThesisLogger("benchmark", tracker_name="bytetrack")
#                logger.log(Frame_Number=1, Time_Sec=0.5, ...)
###############################################################################

import csv
import os


class ThesisLogger:
    """CSV logger that creates the correct file and headers based on test mode.

    Supported test modes:
        - "combined"       : Tracker comparison, Distance estimation & velocity (Chapter 4 & 6)
        - "step_response"  : Yaw PD step-response (Chapter 5)
    """

    HEADERS = {
        "step_response": [
            "Time_Sec",
            "e_x",
            "e_y",
            "e_area",
            "e_mag",
            "v_x",
            "v_z",
            "omega_z",
        ],
        "combined": [
            "Frame_Number",
            "Time_Sec",
            "Processing_Time_ms",
            "FPS",
            "Object_ID",
            "Bbox_X",
            "Bbox_Y",
            "A_real",
            "Distance_Est",
            "v_x",
            "v_z",
            "omega_z",
            "current_pitch_rad",
            "Pipeline_Latency_ms",
        ],
    }

    def __init__(self, test_mode, tracker_name=None, run_name=None):
        """Initialize the logger.

        Args:
            test_mode: One of 'combined', 'step_response'.
            tracker_name: Optional tracker name for combined filenames
                          (e.g., 'bytetrack' → combined_bytetrack.csv).
            run_name: Optional string to group logs into a subfolder (e.g., 'run_002').
        """
        if test_mode not in self.HEADERS:
            raise ValueError(
                f"Unknown test_mode '{test_mode}'. "
                f"Choose from: {list(self.HEADERS.keys())}"
            )

        self.test_mode = test_mode
        self.headers = self.HEADERS[test_mode]

        # Build output path: graphs_generation/logs/[run_name]/<filename>.csv
        output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
        if run_name:
            output_dir = os.path.join(output_dir, run_name)
        os.makedirs(output_dir, exist_ok=True)

        if tracker_name:
            filename = f"{test_mode}_{tracker_name}.csv"
        else:
            filename = f"{test_mode}.csv"

        self.filepath = os.path.join(output_dir, filename)
        self._file = open(self.filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.headers)
        self._file.flush()

        print(f"[ThesisLogger] Mode: {test_mode} -> {self.filepath}")

    def log(self, **kwargs):
        """Write one CSV row. Pass column names as keyword arguments.

        Unknown keys are silently ignored; missing keys become empty strings.
        The file is flushed after every write so data survives Ctrl+C.
        """
        row = [kwargs.get(h, "") for h in self.headers]
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        """Flush and close the CSV file."""
        if self._file and not self._file.closed:
            self._file.close()
            print(f"[ThesisLogger] Saved: {self.filepath}")
