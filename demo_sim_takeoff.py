#########################

# Author: Luca Boninsegna
# Date:   25/03/26
# Descr:  Simple takeoff at a given altitude, then landing

#########################

from pymavlink import mavutil
import time


# Virtual SITL Connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Waiting for a valid MAVLink heartbeat packet
print("Bridge open. Listening for ArduPilot heartbeat...")
master.wait_heartbeat()

# Connection confirmation
print("TARGET ACQUIRED: Heartbeat Received!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")


print("\n--- INITIATING AUTOMATED FLIGHT SEQUENCE ---")

# Switch to GUIDED Mode
mode_id = master.mode_mapping()['GUIDED']
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("Command Sent: Switched to GUIDED mode")
time.sleep(2)

# Force RC channels (Roll, Pitch, Throttle, Yaw) to neutral
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 1500, 1000, 1500, 0, 0, 0, 0
)
time.sleep(1)

# Arm the motors
master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, # Confirmation (0 = first attempt)
    1, 0, 0, 0, 0, 0, 0 
)
print("Command Sent: Motors ARMED")
time.sleep(2)

# Takeoff command
target_altitude = 10 # in meters
master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 
    0, 0, 0, 0, 0, 0, target_altitude
)
print(f"Command Sent: TAKEOFF to {target_altitude} meters")

# Drone hovers at the target altitude
print("Hovering for few seconds...")
time.sleep(20)

# Land command
print("Command Sent: Initiating LAND sequence")
master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 
    0, 0, 0, 0, 0, 0, 0
)
