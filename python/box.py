import modern_robotics as mr
import numpy as np

# -----------------------------
# Robot definition (yours)
# -----------------------------
Slist = np.array([[0,   0,      0,      0,  0,      -1],
                  [0,   -1,     -1,     0,  1,      0],
                  [1,  0,      0,      1,  0,      0],
                  [0,   85.9,   385.9,  30, -579.3, 0],
                  [0,   0,      0,      0,  0,      -648.9],
                  [0,   0,      0,      0,  8.6,    25.3]])

M = np.array([[1, 0, 0, 15.25],
              [0, 1, 0, 25.75],
              [0, 0, 1, 703.9],
              [0, 0, 0, 1]])

joint_limits = [(-1.57, 1.57)] * 6

# -----------------------------
# Helper functions
# -----------------------------
def within_limits(theta):
    """Check joint limits"""
    for i in range(6):
        if not (joint_limits[i][0] <= theta[i] <= joint_limits[i][1]):
            return False
    return True


def IK_space(T, theta_init):
    """Inverse kinematics with joint-limit checking"""
    eomg = 1e-3
    ev = 1e-3

    theta, success = mr.IKinSpace(
        Slist, M, T, theta_init, eomg, ev
    )

    # if not success or not within_limits(theta):
    if not success:
        raise ValueError("IK failed or joint limits exceeded")

    return theta


# -----------------------------
# Define task-space poses
# -----------------------------

# Home configuration
theta_home = np.ones(6) / 10 # small angles near zero
T_home = mr.FKinSpace(M, Slist, theta_home)

# Pick pose (example)
T_start = np.array([
    [-1, 0, 0, -350],
    [0, 1, 0, 0],
    [0, 0, -1, 67],
    [0, 0, 0, 1]
])

L = -100 # mm box size
z_offset = -16 #mm on the left side

T_1 = T_start.copy()
T_1[0:3, 3] += np.array([L, 0, 0])  # Move in L,0

T_2 = T_start.copy()
T_2[0:3, 3] += np.array([L, L, z_offset // 2])  # Move in L,L
T_3 = T_start.copy()
T_3[0:3, 3] += np.array([0, L, z_offset])  # Move in 0,L

T_4 = T_start.copy() # back to start, 0,0
T_4[0:3, 3] += np.array([0, 0, z_offset])  # Move in 0,0,z_offset


# Approach offset (hover above object)
hover_height = 20  # mm

def hover_pose(T):
    T_hover = T.copy()
    T_hover[2, 3] += hover_height
    return T_hover


# -----------------------------
# Generate Cartesian trajectories
# -----------------------------
Tf = 2.0          # seconds
N = 100           # trajectory points
method = 5        # quintic time scaling

traj = []

# Home → hover above pick
traj += mr.CartesianTrajectory(
    T_home, hover_pose(T_start), Tf, N, method
)

# Hover → start
traj += mr.CartesianTrajectory(
    hover_pose(T_start), T_start, Tf/2, N//2, method
)

# Start → box corners
traj += mr.CartesianTrajectory(
    T_start, T_1, Tf, N, method
)
traj += mr.CartesianTrajectory(
    T_1, T_2, Tf, N, method
)
traj += mr.CartesianTrajectory(
    T_2, T_3, Tf, N, method
)
traj += mr.CartesianTrajectory(
    T_3, T_4, Tf, N, method
)  
traj += mr.CartesianTrajectory(
    T_4, hover_pose(T_4), Tf/2, N//2, method
)

# -----------------------------
# Solve IK for each pose
# -----------------------------
theta_traj = []
theta_prev = theta_home.copy()

for T in traj:
    theta_sol = IK_space(T, theta_prev)
    theta_traj.append(theta_sol)
    theta_prev = theta_sol




theta_traj = np.array(theta_traj)

# -----------------------------
# Result
# -----------------------------
print("Trajectory length:", theta_traj.shape[0])
print("Final joint angles (rad):")
print(theta_traj[-1])

# Save trajectory to file
np.save("theta_traj.npy", theta_traj)