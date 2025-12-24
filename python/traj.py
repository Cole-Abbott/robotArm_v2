import modern_robotics as mr
import numpy as np

# -----------------------------
# Robot definition (yours)
# -----------------------------
Slist = np.array([[0,   0,      0,      0,  0,      -1],
                  [0,   -1,     -1,     0,  1,      0],
                  [-1,  0,      0,      1,  0,      0],
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
T_pick = np.array([
    [-1, 0, 0, -350],
    [0, 1, 0, 0],
    [0, 0, -1, 50],
    [0, 0, 0, 1]
])

T_place = np.array([
    [-1, 0, 0, -350],
    [0, 1, 0, 50],
    [0, 0, -1, 50],
    [0, 0, 0, 1]
])


# Place pose (example)
# T_place = np.array([
#     [0, 1, 0, 0],
#     [1, 0, 0, -350],
#     [0, 0, -1, 100],
#     [0, 0, 0, 1]
# ])

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
N = 50           # trajectory points
method = 5        # quintic time scaling

traj = []

# Home → hover above pick
traj += mr.CartesianTrajectory(
    T_home, hover_pose(T_pick), Tf, N, method
)

# Hover → pick
traj += mr.CartesianTrajectory(
    hover_pose(T_pick), T_pick, Tf/2, N//2, method
)

# Pick → hover (object grasped)
traj += mr.CartesianTrajectory(
    T_pick, hover_pose(T_pick), Tf/2, N//2, method
)

# Hover pick → hover place
traj += mr.CartesianTrajectory(
    hover_pose(T_pick), hover_pose(T_place), Tf, N, method
)

# Hover → place
traj += mr.CartesianTrajectory(
    hover_pose(T_place), T_place, Tf/2, N//2, method
)

# Place → hover
traj += mr.CartesianTrajectory(
    T_place, hover_pose(T_place), Tf/2, N//2, method
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