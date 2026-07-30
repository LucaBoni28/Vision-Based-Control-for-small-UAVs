###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Shared utilities for SITL test scripts.
#         Provides: SITL connection, automated arm+takeoff, CSV logging.
###############################################################################

import os
import sys
import csv
import time

# Add parent directory to path so we can import classes.*
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from classes.config import AppConfig
from classes.flight_controller import FlightController


# ─── CSV Logger ──────────────────────────────────────────────────────────────

class CSVLogger:
    """
    Simple CSV logger that writes to graphs_generation/logs/<filename>.csv.
    Follows the same pattern as ThesisLogger on the master branch.
    """

    def __init__(self, filename: str, headers: list):
        """
        Args:
            filename: Name of the CSV file (without directory path, e.g. 'test_1_step_vx.csv')
            headers: List of column header strings
        """
        import __main__
        if hasattr(__main__, '__file__'):
            caller_dir = os.path.dirname(os.path.abspath(__main__.__file__))
        else:
            caller_dir = os.getcwd()
            
        output_dir = os.path.join(caller_dir, "logs")
        os.makedirs(output_dir, exist_ok=True)

        self.filepath = os.path.join(output_dir, filename)
        self._file = open(self.filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(headers)
        self._file.flush()
        print(f"CSV Logger: writing to {self.filepath}")

    def log(self, *values):
        """Write a single row of values."""
        self._writer.writerow(values)
        self._file.flush()

    def close(self):
        """Close the CSV file."""
        self._file.close()
        print(f"CSV Logger: closed {self.filepath}")


# ─── SITL Connection ─────────────────────────────────────────────────────────

def load_config(config_path: str = None) -> AppConfig:
    """Load the application config, defaulting to classes/config.yaml."""
    if config_path is None:
        project_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
        config_path = os.path.join(project_root, "classes", "config.yaml")
    return AppConfig.load(config_path)


def sitl_connect(config: AppConfig = None, config_path: str = None) -> FlightController:
    """
    Create and connect a FlightController using the config.yaml settings.

    Args:
        config: Pre-loaded AppConfig. If None, loads from config_path.
        config_path: Path to config.yaml (used only if config is None).

    Returns:
        Connected FlightController instance.
    """
    if config is None:
        config = load_config(config_path)

    flight = FlightController(config.mavlink)
    flight.connect()
    return flight


def sitl_arm_and_takeoff(flight: FlightController, target_alt: float = 10.0,
                         timeout: float = 30.0) -> bool:
    """
    Arm the drone and take off to the specified altitude.
    Waits until the drone reaches the target altitude (±1m) or times out.

    Args:
        flight: Connected FlightController instance.
        target_alt: Target altitude in meters above home.
        timeout: Maximum time to wait for altitude in seconds.

    Returns:
        True if altitude reached, False if timed out.
    """
    print(f"\n--- AUTOMATED ARM & TAKEOFF to {target_alt}m ---")

    # Switch to GUIDED mode
    flight.set_flight_mode("GUIDED")
    time.sleep(2)

    # Get initial altitude to handle cases where SITL relative_alt is not 0
    flight.poll_heartbeat()
    initial_alt = flight.poll_relative_alt() or 0.0
    target_abs_alt = initial_alt + target_alt
    print(f"Targeting absolute altitude: {target_abs_alt:.1f}m (Initial: {initial_alt:.1f}m)")
    
    # If already flying and armed, skip takeoff
    if initial_alt > 2.0 and flight.is_armed():
        print("Drone is already flying and armed. Skipping takeoff phase.")
        return True

    # Arm the motors
    from pymavlink import mavutil
    print("Arming motors...")
    flight.master.mav.command_long_send(
        flight.target_system,
        flight.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1, 0, 0, 0, 0, 0, 0
    )
    if getattr(flight, 'telemetry_output', None):
        try:
            flight.telemetry_output.mav.command_long_send(
                0, 0,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
        except Exception:
            pass

    print("Arm command sent. Waiting for motors...")
    time.sleep(3)

    # Send takeoff command using target_abs_alt
    flight.master.mav.command_long_send(
        flight.target_system,
        flight.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0, 0, 0, target_abs_alt
    )
    if getattr(flight, 'telemetry_output', None):
        try:
            flight.telemetry_output.mav.command_long_send(
                0, 0,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, target_abs_alt
            )
        except Exception:
            pass
    print(f"Takeoff command sent: climbing to {target_abs_alt:.1f}m")

    # Wait for altitude
    start_time = time.time()

    while time.time() - start_time < timeout:
        flight.poll_heartbeat()
        alt = flight.poll_relative_alt()
        if alt is not None:
            print(f"  Altitude: {alt:.1f}m / {target_abs_alt:.1f}m", end="\r")
            if alt >= target_abs_alt - 1.0:
                print(f"\n  Reached target altitude: {alt:.1f}m")
                # Let it stabilize
                time.sleep(3)
                return True
        time.sleep(0.5)

    print(f"\nWARNING: Takeoff timed out after {timeout}s")
    return False


def wait_for_position_data(flight: FlightController, timeout: float = 10.0) -> bool:
    """
    Wait until LOCAL_POSITION_NED data is available from SITL.

    Args:
        flight: Connected FlightController instance.
        timeout: Maximum time to wait in seconds.

    Returns:
        True if position data is available, False if timed out.
    """
    print("Waiting for LOCAL_POSITION_NED data from SITL...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        flight.poll_heartbeat()
        pos = flight.poll_local_position_ned()
        if pos is not None:
            print(f"  Got NED position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
            return True
        time.sleep(0.2)

    print(f"WARNING: No LOCAL_POSITION_NED data after {timeout}s. "
          "Make sure SITL is running and streaming position data.")
    return False
