#########################

# Author: Luca Boninsegna
# Date:   25/03/26
# Descr:  Reading the telemetry for physical Pixhawk via USB port

#########################

from pymavlink import mavutil
import time
import math


# Define the Hardware Port and Baud Rate
port = '/dev/ttyACM0'  
baud_rate = 115200
print(f"Attempting to open MAVLink bridge on {port}...")

# Initialize the Conection
master = mavutil.mavlink_connection(port, baud=baud_rate)

# Waiting for a valid MAVLink heartbeat packet
print("Bridge open. Listening for ArduPilot heartbeat...")
master.wait_heartbeat()

# Connection confirmation 
print("TARGET ACQUIRED: Heartbeat Received!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")


print("\n--- INITIATING IMU TELEMETRY STREAM ---")

# Set up the Pixhawk to send IMU data
print("Requesting High-Speed Telemetry Stream...")
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, # All data categories
    10, # 10 messages per second
    1   # Start sending
)
print("Pick up the Pixhawk and move it around with your hands!\n")

while(1):
    
    # Select the packet 'ATTITUDE'
    # blocking=True -> wait until next packet
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    
    # Convert radians to degrees
    roll_deg = math.degrees(msg.roll)
    pitch_deg = math.degrees(msg.pitch)
    yaw_deg = math.degrees(msg.yaw)
    
    # Print the physical orientation formatted to 2 decimal places
    print(f"Roll: {roll_deg:5.2f}° | Pitch: {pitch_deg:5.2f}° | Yaw: {yaw_deg:5.2f}°")