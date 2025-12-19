import numpy as np
import modern_robotics as mr

import simulator as sim
import trajectory as tr
import feedback as fb

# Joint Screw Axis
B1 = [0,0,1,0,0.033,0]
B2 = [0,-1,0,-0.5076,0,0]
B3 = [0,-1,0,-0.3526,0,0]
B4 = [0,-1,0,-0.2176,0,0]
B5 = [0,0,1,0,0,0]
Blist = np.array([B1, B2, B3, B4, B5]).T

M_0e = np.array([ # M matrix for robot arm
    [1,0,0,0.033],
    [0,1,0,0],
    [0,0,1,0.6546],
    [0,0,0,1]
])


# input parameters

T_sc_initial = np.array([ # inital cube position
    [1,0,0,-0.5],
    [0,1,0,1],
    [0,0,1,0.025],
    [0,0,0,1]
])

T_sc_goal = np.array([ # final cube position
    [-0.71,-0.71,0,0.5],
    [0.71,-0.71,0,0],
    [0,0,1,0.025],
    [0,0,0,1]
])


T_ce_grasp = np.array([ # to grasp have gripper at 45 degree angle and aligned with {c}
    [-0.71,0,0.71,0.01],
    [0,1,0,0],
    [-0.71,0,-0.71,0],
    [0,0,0,1]
])

T_ce_standoff = np.array([ # to standoff have {e} above {c] by 15cm
    [-0.71,0,0.71,0.01],
    [0,1,0,0],
    [-0.71,0,-0.71,0.15],
    [0,0,0,1]
])

T_ce_grasp = np.array([ # to grasp have gripper at 45 degree angle and aligned with {c}
    [-0.8660254,0,0.5,0.01],
    [0,1,0,0],
    [-0.5,0,-0.8660254,0],
    [0,0,0,1]
])

T_ce_standoff = np.array([ # to grasp have gripper at 45 degree angle and aligned with {c}
    [-0.8660254,0,0.5,0.01],
    [0,1,0,0],
    [-0.5,0,-0.8660254,0.15],
    [0,0,0,1]
])


# 12 vector of initial robot configuration
q = np.array([-0.4,-0.5,-0.0,0.5,-0.5,-0.5,-0.2,0.3,0,0,0,0])

# inital configuration of end-effector
T_se_initial = np.array([
    [0,0,1,0],
    [0,1,0,0],
    [-1,0,0,0.3],
    [0,0,0,1]
])

# calculate the trajectory
print("Generating Trajectory")
traj, gripper = tr.TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_goal, T_ce_grasp, T_ce_standoff)
N = np.shape(traj)[0]

dt = 0.01 # timestep
 # Kp, Ki
gains = [[0.3,0.3,0.5,1,0.5,0.5],0]


maxSpeed = 100
limits = np.array([[1.2,-1.2], # joint 1 limits
                   [-0.1,-1.7], # joint 2 limits
                   [-0.2,-1.7], # joint 3 limits
                   [10,-10], # joint 4 limits
                   [10,-10] # joint 5 limits
                   ])

X_err_int = np.zeros((6)) # integral error twist

# for CSV file
qList = np.zeros((N-1,13))
X_errList = np.zeros((N-1,6))
VList = np.zeros((N-1,6))

# simulate robot using feedback to follow the trajectory
print("Simulating Robot Conrol")
for i in range(N-1):

    # store q in qList
    qList[i][0:12] = q
    qList[i][12] = gripper[i]
   

    # find X from joint vector q
    X = sim.endEffectorPos(q,M_0e,Blist)

    # find desired end-effector twist from feedback controller
    V, X_err, X_err_int = fb.FeedbackControl(X,traj[i],traj[i+1], X_err_int, gains, dt)
    X_errList[i] = X_err
    VList[i] = V

    # find joint velocites to create desired twist
    J = sim.Jacobian(q,M_0e,Blist)
    # qdot = np.linalg.pinv(J) @ V
    qdot = np.linalg.pinv(J, 0.001) @ V


    # check if the next state violates the joint limits
    violations = sim.testJointLimits(q, qdot, dt, maxSpeed, limits)
    
    # if the next state violates the joint limits, set that column of J to 0
    for j in range(5):
        if violations[j]:
            J[:,j+4] = np.zeros(6)
    
    # recalculate qdot
    qdot = np.linalg.pinv(J, 0.01) @ V

    # simulate robot movement 
    q = sim.nextState(q,qdot, dt, maxSpeed)

print("Saving Data")
np.savetxt("newTask.csv", qList, delimiter=",")
np.savetxt("newTask_err.csv", X_errList, delimiter=",")



