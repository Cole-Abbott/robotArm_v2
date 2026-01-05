"""
Interactive keyboard teleop for the robot arm.

Keys:
  Position: W/A/S/D for X/Y (mm), Q/E for Z (mm)
  Rotation (roll/pitch/yaw, rad): I/K for +roll/-roll, J/L for +pitch/-pitch, U/O for +yaw/-yaw

Sends joint angles over WebSocket in the format:
  {"t1": <rad>, ..., "t6": <rad>, "type": 1}

Dependencies (pip): pynput, websocket-client, modern-robotics
Run: python manual.py
"""

import json
import time
from pynput import keyboard
import numpy as np
import websocket
import modern_robotics as mr

# -----------------------------
# Robot definition (6-DOF)
# -----------------------------
Slist = np.array([
    [0, 0, 0, 0, 0, -1],
    [0, -1, -1, 0, 1, 0],
    [-1, 0, 0, 1, 0, 0],
    [0, 85.9, 385.9, 30, -579.3, 0],
    [0, 0, 0, 0, 0, -648.9],
    [0, 0, 0, 0, 8.6, 25.3],
])

M = np.array([
    [1, 0, 0, 15.25],
    [0, 1, 0, 25.75],
    [0, 0, 1, 703.9],
    [0, 0, 0, 1],
])

# Initial joint guess (small non-zero to avoid singularities)
theta_current = np.ones(6) * 0.1
theta_limit = [(-1.57, 1.57)] * 6


T_start = np.array([
    [-1, 0, 0, -350],
    [0, 1, 0, 0],
    [0, 0, -1, 50],
    [0, 0, 0, 1]
])

# generate a traj from theta_current to the ik of T_start
T_home = mr.FKinSpace(M, Slist, theta_current)
# Home → start
Tf = 2.0  # seconds
N = 60    # steps
method = 5  # cubic time-scaling
traj = mr.CartesianTrajectory(
    T_home, T_start, Tf, N, method
)


# IK tolerances
EOMG = 1e-3
EV = 1e-3

# WebSocket target
ESP32_WEBSOCKET_URL = "ws://192.168.1.215/ws"

# Step sizes
LINEAR_STEP = 2.0   # mm per keypress cycle
ANG_STEP = 0.05     # rad per keypress cycle

# Track pressed keys
pressed = set()


def _key_to_char(key):
    try:
        return key.char.lower()
    except AttributeError:
        return None


def on_press(key):
    c = _key_to_char(key)
    if c:
        pressed.add(c)


def on_release(key):
    c = _key_to_char(key)
    if c and c in pressed:
        pressed.remove(c)


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Compute rotation matrix from roll, pitch, yaw (XYZ intrinsic)."""
    R_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )
    R_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )
    R_z = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    return R_z @ R_y @ R_x


def build_target_T(pos: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[0:3, 0:3] = rpy_to_rot(rpy[0], rpy[1], rpy[2])
    T[0:3, 3] = pos
    return T


def send_joints(ws, theta: np.ndarray):
    # check joint limits
    theta = np.clip(theta, [lim[0] for lim in theta_limit], [lim[1] for lim in theta_limit])
    
    payload = {
        "t1": float(theta[0]),
        "t2": float(theta[1]),
        "t3": float(theta[2]),
        "t4": float(theta[3]),
        "t5": float(theta[4]),
        "t6": float(theta[5]),
        "type": 1,
    }
    ws.send(json.dumps(payload))


def main():
    global theta_current

    ws = websocket.WebSocket()
    ws.connect(ESP32_WEBSOCKET_URL)
    print(f"Connected to {ESP32_WEBSOCKET_URL}")

    # execute initialization traj
    for T in traj:
        theta_sol, success = mr.IKinSpace(Slist, M, T, theta_current, EOMG, EV)
        if success:
            theta_current = theta_sol
            send_joints(ws, theta_current)
            time.sleep(0.1)
        else:
            print("IK failed during initialization traj; skipping send")
    
     # Start pose from forward kinematics of initial joints
    T_current = mr.FKinSpace(M, Slist, theta_current)
    pos = T_current[0:3, 3].copy()
    rpy = np.array([0.0, 0.0, 0.0])

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("Controls: W/A/S/D = X/Y, Q/E = Z, I/K roll, J/L pitch, U/O yaw. Ctrl+C to exit.")

    try:
        while True:
            updated = False

            # Position keys
            if 'w' in pressed:
                pos[0] += LINEAR_STEP; updated = True
            if 's' in pressed:
                pos[0] -= LINEAR_STEP; updated = True
            if 'd' in pressed:
                pos[1] += LINEAR_STEP; updated = True
            if 'a' in pressed:
                pos[1] -= LINEAR_STEP; updated = True
            if 'q' in pressed:
                pos[2] += LINEAR_STEP; updated = True
            if 'e' in pressed:
                pos[2] -= LINEAR_STEP; updated = True

            # Orientation keys
            if 'i' in pressed:
                rpy[0] += ANG_STEP; updated = True
            if 'k' in pressed:
                rpy[0] -= ANG_STEP; updated = True
            if 'j' in pressed:
                rpy[1] += ANG_STEP; updated = True
            if 'l' in pressed:
                rpy[1] -= ANG_STEP; updated = True
            if 'u' in pressed:
                rpy[2] += ANG_STEP; updated = True
            if 'o' in pressed:
                rpy[2] -= ANG_STEP; updated = True

            if updated:
                T_target = build_target_T(pos, rpy)
                theta_sol, success = mr.IKinSpace(Slist, M, T_target, theta_current, EOMG, EV)
                if success:
                    theta_current = theta_sol
                    send_joints(ws, theta_current)
                    print(f"Sent joints: {theta_current}")
                else:
                    print("IK failed for target pose; skipping send")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        listener.stop()
        ws.close()
        print("WebSocket closed")


if __name__ == "__main__":
    main()
