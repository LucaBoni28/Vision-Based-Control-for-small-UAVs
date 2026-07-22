###############################################################################
# Author: Luca Boninsegna
# Date:   23/07/26
# Descr:  Test 2 - Yaw PD Controller Step-Response via SITL
#         Injects a fake horizontal error and records how the controller
#         converges, producing the overshoot/settling-time graphs for Ch.5.
#
# Prerequisites:
#   1. Start ArduPilot SITL  (sim_vehicle.py or Mission Planner simulation)
#   2. Connect Mission Planner on port 14550  (GCS)
#   3. Arm & take off in GUIDED mode
#   4. Run this script:  python step_response_test.py
#
# How it works:
#   - At t=0 a virtual target appears at STEP_SIZE (e.g. 0.5 = 50% right)
#   - The PD controller commands omega_z to yaw toward it
#   - SITL rotates the virtual drone; we read back the yaw from ATTITUDE
#   - We subtract the yaw displacement from the fake error, closing the loop
#   - We log e_x, omega_z vs time -> step-response curve
###############################################################################

import time
import math
from pymavlink import mavutil
from graphs_generation.thesis_logger import ThesisLogger


# ---- Controller gains  (MUST match tracking.py) ----------------------------
K_p_yaw = 1.0      # Proportional gain for yaw rate
K_d_yaw = 0.05     # Derivative gain for yaw rate

# ---- Camera parameters -----------------------------------------------------
# IMX219 CSI camera at 1280x960 -> HFOV ~ 62 deg
CAMERA_HFOV_DEG = 62.2
CAMERA_HALF_HFOV_RAD = math.radians(CAMERA_HFOV_DEG / 2)   # ~0.54 rad

# ---- Test parameters --------------------------------------------------------
STEP_SIZE = 0.5     # Normalised error: target at 50% of frame width from centre
TEST_DURATION = 15  # seconds
LOOP_RATE = 30      # Hz  (matches camera frame rate)


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def main():
    print("=" * 60)
    print("  TEST 2 — Yaw PD Controller Step Response (SITL)")
    print("=" * 60)
    print(f"  Step size       : {STEP_SIZE}  (normalised)")
    print(f"  K_p_yaw         : {K_p_yaw}")
    print(f"  K_d_yaw         : {K_d_yaw}")
    print(f"  Camera half-FOV : {math.degrees(CAMERA_HALF_HFOV_RAD):.1f} deg")
    print(f"  Duration        : {TEST_DURATION} s  @  {LOOP_RATE} Hz")
    print("=" * 60)

    # ---- MAVLink connection (same as tracking.py) ---------------------------
    print("\nConnecting to SITL...")
    master = mavutil.mavlink_connection(
        'udpin:0.0.0.0:14551',
        source_system=255,
        source_component=191,
    )
    master.wait_heartbeat()
    print(f"Heartbeat received  —  System {master.target_system}, "
          f"Component {master.target_component}")

    # Request ATTITUDE stream at the loop rate
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        LOOP_RATE,
        1,
    )

    # ---- Capture initial yaw ------------------------------------------------
    print("Waiting for ATTITUDE data...")
    initial_yaw = None
    for _ in range(100):
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg:
            initial_yaw = msg.yaw
            break

    if initial_yaw is None:
        print("ERROR: No ATTITUDE message. Is SITL running and the drone armed/airborne?")
        return

    print(f"Initial yaw: {math.degrees(initial_yaw):.1f} deg")
    print(f"Injecting step error = {STEP_SIZE} ...\n")
    print("Press Ctrl+C to stop early.\n")

    # ---- Logger -------------------------------------------------------------
    logger = ThesisLogger("step_response")

    # ---- Control loop -------------------------------------------------------
    prev_e_x = STEP_SIZE    # Error at t=0 (before drone moves)
    prev_time = time.time()
    start_time = time.time()

    try:
        while (time.time() - start_time) < TEST_DURATION:
            loop_start = time.time()

            # Read current yaw from SITL
            msg = master.recv_match(type='ATTITUDE', blocking=False)
            if msg is None:
                # No new ATTITUDE yet — sleep and retry
                time.sleep(1.0 / LOOP_RATE)
                continue

            current_yaw = msg.yaw

            # How far has the drone rotated since t=0?
            yaw_delta = normalize_angle(current_yaw - initial_yaw)

            # Virtual error: target at +STEP_SIZE, minus the drone's rotation
            e_x = STEP_SIZE - (yaw_delta / CAMERA_HALF_HFOV_RAD)
            e_x = max(-1.0, min(1.0, e_x))       # Clamp to sensor range

            e_y_comp = 0.0                         # No vertical component
            e_mag = abs(e_x)

            # ---- PD controller (identical to tracking.py) -------------------
            current_time = time.time()
            dt = current_time - prev_time

            if 0 < dt < 0.5:
                derivative_x = (e_x - prev_e_x) / dt
            else:
                derivative_x = 0.0

            omega_z = K_p_yaw * e_x + K_d_yaw * derivative_x

            # Dead-zone (same threshold as tracking.py)
            if abs(omega_z) < 0.03:
                omega_z = 0.0

            v_z = 0.0   # No vertical movement in this test

            # ---- Send yaw-rate command (no translation) ---------------------
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000011111000111,
                0, 0, 0,
                0.0, 0.0, 0.0,       # vx, vy, vz
                0, 0, 0,
                0, omega_z,           # yaw, yaw_rate
            )

            # ---- Log --------------------------------------------------------
            elapsed = current_time - start_time
            logger.log(
                Time_Sec=round(elapsed, 3),
                e_x=round(e_x, 6),
                e_y_comp=round(e_y_comp, 6),
                e_mag=round(e_mag, 6),
                omega_z=round(omega_z, 6),
                v_z=round(v_z, 6),
            )

            # Console feedback
            print(f"t={elapsed:5.1f}s | e_x={e_x:+8.4f} | "
                  f"omega_z={omega_z:+8.4f} rad/s | "
                  f"delta_yaw={math.degrees(yaw_delta):+6.1f} deg")

            # Update state
            prev_e_x = e_x
            prev_time = current_time

            # Rate-limit to LOOP_RATE Hz
            elapsed_loop = time.time() - loop_start
            sleep_time = max(0, (1.0 / LOOP_RATE) - elapsed_loop)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")

    # ---- Stop the drone yaw -------------------------------------------------
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,
        0, 0, 0,
        0.0, 0.0, 0.0,
        0, 0, 0,
        0, 0.0,
    )

    logger.close()
    print("\nStep-response test complete.")


if __name__ == "__main__":
    main()
