from pymavlink import mavutil

# 1. Define the Hardware Port and Baud Rate
# Update this string to match your exact physical connection port.
# 115200 is the standard baud rate (speed) for MAVLink telemetry.
#port = '/dev/ttyACM0'  
#baud_rate = 115200

#print(f"Attempting to open MAVLink bridge on {port}...")

# Initialize the Connection
#master = mavutil.mavlink_connection(port, baud=baud_rate)

# Virtual SITL Connection
# 127.0.0.1 is your "localhost" loopback address. 
# 14550 is the standard ArduPilot UDP telemetry port.
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