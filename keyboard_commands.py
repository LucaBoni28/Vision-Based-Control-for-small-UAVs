###############################################################################
# Author: Luca Boninsegna
# Date:   21/04/26
# Descr:  Control a virtual drone in mission planner with the keyboard
#         W/S : Move Forward/Backward
#         A/D : Move Left/Right
#         UP/DOWN : Change Altitude
#         Hold 'Q' to Quit
###############################################################################

import time
import keyboard
from pymavlink import mavutil
import os
import atexit
import sys
import termios

# The Unix Terminal Bypass
print("Silencing terminal output...")
os.system("stty -echo") # Disables the visual typing echo in the terminal

def restore_terminal():
    termios.tcflush(sys.stdin, termios.TCIFLUSH)
    os.system("stty echo")
    print("\nTerminal echo restored")
    
atexit.register(restore_terminal)

# Establish the UDP Server on the Jetson
print("Waiting for WSL simulation to push heartbeat...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
master.wait_heartbeat()
print(f"Target locked! System: {master.target_system}, Component: {master.target_component}")

# Define the Velocity Command Function
def send_velocity(vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111, # Explicit binary mask for velocities and yaw rate
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

print("--- COMMANDS LIST---")
print("W/S : Move Forward/Backward")
print("A/D : Move Left/Right")
print("UP/DOWN : Change Altitude")
print("Hold 'Q' to Quit")

# The Real-Time Polling Loop
try:
    while True:
        # Default state is hovering (zero velocity)
        vx, vy, vz = 0.0, 0.0, 0.0

        # Read the keyboard state passively
        if keyboard.is_pressed('w'): vx = 2.0   
        if keyboard.is_pressed('s'): vx = -2.0  
        if keyboard.is_pressed('d'): vy = 2.0   
        if keyboard.is_pressed('a'): vy = -2.0  
        
        if keyboard.is_pressed('up'): vz = -1.0   
        if keyboard.is_pressed('down'): vz = 1.0  
        
        if keyboard.is_pressed('q'):
            print("Terminating teleoperation...")
            break

        # Transmit the calculated vectors to the flight controller
        send_velocity(vx, vy, vz)
        
        # Maintain a 10Hz control loop
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Emergency stop activated.")