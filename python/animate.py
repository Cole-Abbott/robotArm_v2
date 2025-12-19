import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --------------------------------
# Robot definition
# --------------------------------
Slist = np.array([[0,   0,      0,      0,  0,      1],
                  [0,   -1,     -1,     0,  1,      0],
                  [-1,  0,      0,      1,  0,      0],
                  [0,   85.9,   385.9,  30, -579.3, 0],
                  [0,   0,      0,      0,  0,      648.9],
                  [0,   0,      0,      0,  8.6,    -25.3]])

M = np.array([[1, 0, 0, 15.25],
              [0, 1, 0, 25.75],
              [0, 0, 1, 703.9],
              [0, 0, 0, 1]])

# --------------------------------
# Load trajectory
# --------------------------------
theta_traj = np.load("theta_traj.npy")   # shape (N, 6)

# --------------------------------
# Coordinate frame drawing helper
# --------------------------------
def draw_frame(ax, T, length=80, lw=3):
    """Draws a coordinate frame given SE(3) matrix T"""
    p = T[:3, 3]
    R = T[:3, :3]

    axes = [
        (R[:, 0], 'r'),  # x-axis
        (R[:, 1], 'g'),  # y-axis
        (R[:, 2], 'b')   # z-axis
    ]

    lines = []
    for axis, color in axes:
        line, = ax.plot(
            [p[0], p[0] + length * axis[0]],
            [p[1], p[1] + length * axis[1]],
            [p[2], p[2] + length * axis[2]],
            color=color,
            lw=lw
        )
        lines.append(line)

    return lines


# --------------------------------
# Matplotlib animation
# --------------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)
ax.set_zlim(0, 800)

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_zlabel("Z (mm)")

# Draw fixed space frame
space_frame = draw_frame(ax, np.eye(4), length=120, lw=4)

# Initialize EE frame
ee_lines = draw_frame(ax, np.eye(4))

def update(frame):
    global ee_lines

    # Remove previous EE frame
    for line in ee_lines:
        line.remove()

    # Compute EE pose
    theta = theta_traj[frame]
    T_ee = mr.FKinSpace(M, Slist, theta)

    # Draw new EE frame
    ee_lines = draw_frame(ax, T_ee)

    return ee_lines

ani = FuncAnimation(
    fig,
    update,
    frames=len(theta_traj),
    interval=50,
    blit=False
)

plt.show()
