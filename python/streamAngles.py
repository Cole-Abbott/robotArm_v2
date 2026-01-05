# open theta_traj.npy and plot joint angles over time
import numpy as np
import matplotlib.pyplot as plt 
import websocket

ESP32_WEBSOCKET_URL = "ws://192.168.1.215/ws"

# Load trajectory
theta_traj = np.load("theta_traj.npy")   # shape (N, 6) 

# connect to websocket server

ws = websocket.WebSocket()
ws.connect(ESP32_WEBSOCKET_URL)
print("Connected to ESP32 WebSocket server")

# skip the fisrt 150 frames to avoid the initialization part
theta_traj = theta_traj[150:]

# Send joint angles over websocket in the json format {"t1": <rad>, "t2": <rad>, "t3": <rad>, "t4": <rad>, "t5": <rad>, "t6": <rad>, "type": 1}
for theta in theta_traj:
    message = {
        "t1": float(theta[0]),
        "t2": float(theta[1]),
        "t3": float(theta[2]),
        "t4": float(theta[3]),
        "t5": float(theta[4]),
        "t6": float(theta[5]),
        "type": 1
    }
    ws.send(str(message))
    print(f"Sent: {message}")
    # 
    plt.pause(0.2)

ws.close()
print("WebSocket connection closed")