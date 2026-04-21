import time
import keyboard
from pymavlink import mavutil

# 1. Establish the TCP connection to SITL
print("Connecting to virtual drone...")
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
master.wait_heartbeat()
print(f"Target locked! System: {master.target_system}")

# 2. Define the Velocity Command Function
def send_velocity(vx, vy, vz):
    """ Command the drone using velocities in meters per second """
    # Bitmask 3527 (0b0000111111000111) strictly enables vx, vy, vz
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        3527,    # Velocity bitmask
        0, 0, 0, # Positions (Ignored)
        vx, vy, vz,
        0, 0, 0, # Accelerations (Ignored)
        0, 0     # Yaw rates (Ignored)
    )

print("--- TELEOPERATION ACTIVE ---")
print("W/S : Move Forward/Backward")
print("A/D : Move Left/Right")
print("UP/DOWN : Change Altitude")
print("Hold 'q' to Quit")

# 3. The Real-Time Control Loop
try:
    while True:
        # Default state is hovering (zero velocity)
        vx, vy, vz = 0.0, 0.0, 0.0

        # Read the keyboard state
        if keyboard.is_pressed('w'): vx = 2.0   # 2 m/s Forward
        if keyboard.is_pressed('s'): vx = -2.0  # 2 m/s Backward
        if keyboard.is_pressed('d'): vy = 2.0   # 2 m/s Right
        if keyboard.is_pressed('a'): vy = -2.0  # 2 m/s Left
        
        if keyboard.is_pressed('up'): vz = -1.0   # 1 m/s Up (NED frame is Negative Z)
        if keyboard.is_pressed('down'): vz = 1.0  # 1 m/s Down
        
        if keyboard.is_pressed('q'):
            print("Terminating")
            break

        # Transmit the calculated vectors to the flight controller
        send_velocity(vx, vy, vz)
        
        # Maintain a 10Hz control loop (standard for MAVLink teleoperation)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Emergency stop activated.")