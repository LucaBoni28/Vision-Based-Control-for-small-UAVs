from pymavlink import mavutil
import time

# Define the Hardware Port and Baud Rate
#port = '/dev/ttyACM0'  
#baud_rate = 115200
#print(f"Attempting to open MAVLink bridge on {port}...")
# Initialize the Conection
#master = mavutil.mavlink_connection(port, baud=baud_rate)

# Virtual SITL Connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

print("Bridge open. Listening for ArduPilot heartbeat...")

# 3. Wait for the Pulse
# This function physically blocks the code from moving forward until 
# a valid MAVLink heartbeat packet is successfully decoded.
master.wait_heartbeat()

# Confirm Telemetry
print("✅ TARGET ACQUIRED: Heartbeat Received!")
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

# Arm the Motors (MAV_CMD_COMPONENT_ARM_DISARM)
master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, # Confirmation (0 = first attempt)
    1, 0, 0, 0, 0, 0, 0 
)
print("Command Sent: Motors ARMED")
time.sleep(2)

# Takeoff Command (MAV_CMD_NAV_TAKEOFF)
target_altitude = 11 # in meters
master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 
    0, 0, 0, 0, 0, 0, target_altitude
)
print(f"Command Sent: TAKEOFF to {target_altitude} meters")

# Drone hovers at the target altitude
print("Hovering for 10 seconds...")
time.sleep(30)

# Command Land (MAV_CMD_NAV_LAND)
print("Command Sent: Initiating LAND sequence")
master.mav.command_long_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 
    0, 0, 0, 0, 0, 0, 0
)
