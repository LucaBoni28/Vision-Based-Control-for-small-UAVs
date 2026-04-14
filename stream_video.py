###############################################################################
# Author: Luca Boninsegna
# Date:   14/04/26
# Descr:  Receives the processed video stream from the Jetson Orin NX and displays it on the Ground Station.
###############################################################################

import cv2
import socket
import numpy as np
import struct

HOST = '0.0.0.0'
PORT = 5005

# Setup TCP Server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
print("Waiting for Jetson to connect...")

conn, addr = s.accept()
print(f"Connected by {addr}")

data = b""
payload_size = struct.calcsize(">L") # Unsigned long integer (4 bytes)

while True:
    # Retrieve the message size
    while len(data) < payload_size:
        packet = conn.recv(4096)
        if not packet: break
        data += packet
    if not data: break
    
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    
    # Retrieve the actual frame data
    while len(data) < msg_size:
        data += conn.recv(4096)
        
    frame_data = data[:msg_size]
    data = data[msg_size:]
    
    # Decode and Display
    frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is not None:
        window_name = "Jetson Tracking Stream"
        
        # Unlock the window size constraints (run this once)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        # Resize the window to a manageable resolution
        cv2.resizeWindow(window_name, 1280, 720)
        
        # Move the window to the top-left corner of your screen
        # cv2.moveWindow(window_name, 50, 50)
        
        # Display the frame inside the newly sized window
        cv2.imshow(window_name, frame)
        
    if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) == -1:
        break