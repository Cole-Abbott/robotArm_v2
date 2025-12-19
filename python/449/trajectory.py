# trajectory.py
# by Cole Abbott
# ME-449 Final Project Milestone 2

import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, 
                        T_ce_grasp, T_ce_standoff, k=1):
    """Generate the reference trajectory for the end-effector frame {e} by 
    concatanating 8 trajectory segments

    :param T_se_initial: Initial end-effector configuration
    :param T_sc_initial: Initial cube configuration
    :param T_sc_final: Desired cube configuration
    :param T_ce_grasp: end-effector configuration when grasping the cube, 
    relative to the cube
    :param T_ce_standoff: end-effector configuration before lowering onto the cube, 
    relative to the cube
    :param k: number of configurations per 0.01 seconds

    :return traj: N x 4 x 4 list of transformation matricies 
    :return gripper: N x 1 list of gripper states

    The functions uses Modern Robotics Trajectorys
    The end effector will pass through the following states:

    T0: Initial Configuration
    T1: Standoff position above inital cube
    T2: Grasp position
    T3: Close Gripper
    T4: Standoff position above inital cube
    T5: Standoff position above final cube
    T6: Lower cube to final position
    T7: Open Gripper
    T8: Standoff position above final cube
    """

    # define each setpoint in the trajectory as a T_se transformation
    T0 = T_se_initial
    T1 = T_sc_initial @ T_ce_standoff
    T2 = T_sc_initial @ T_ce_grasp
    T3 = T_sc_initial @ T_ce_grasp # same as T2, wait here to close gripper
    T4 = T_sc_initial @ T_ce_standoff # same at T1
    T5 = T_sc_final @ T_ce_standoff
    T6 = T_sc_final @ T_ce_grasp
    T7 = T_sc_final @ T_ce_grasp # same as T5, wait here to open gripper
    T8 = T_sc_final @ T_ce_standoff # same as T5

    gripDelay = 0.625
    timeScaleing = 3 # cubic timescaleing

    # speeds
    standoffSpeed = 0.025 
    movementSpeed = 0.1

    # initial to standoff
    dist_01 = np.linalg.norm(T0[0:3,3] - T1[0:3,3])
    T_01 = dist_01 / movementSpeed
    N_01 = int(T_01 * 100 * k)
    traj = mr.ScrewTrajectory(T0, T1, T_01, N_01, timeScaleing)
    gripper = np.zeros(N_01)

    # standoff to grasp
    dist_12 = np.linalg.norm(T1[0:3,3] - T2[0:3,3])
    T_12 = dist_12 / standoffSpeed
    N_12 = int(T_12 * 100 * k)
    traj.extend(mr.ScrewTrajectory(T1, T2, T_12, N_12, timeScaleing))
    gripper = np.concatenate([gripper, np.zeros(N_12)])

    # pause for grasp
    T_23 = gripDelay
    N_23 = int(T_23 * 100 * k)
    traj.extend(mr.ScrewTrajectory(T2, T3, T_23, N_23, timeScaleing))
    gripper = np.concatenate([gripper, np.ones(N_23)])

    # grasp to standoff
    dist_34 = np.linalg.norm(T3[0:3,3] - T4[0:3,3])
    T_34 = dist_34 / standoffSpeed
    N_34 = int(T_34 * 100 * k)
    traj.extend(mr.ScrewTrajectory(T3, T4, T_34, N_34, timeScaleing))
    gripper = np.concatenate([gripper, np.ones(N_34)])

    # major movement
    dist_45 = np.linalg.norm(T4[0:3,3] - T5[0:3,3])
    T_45 = dist_45 / movementSpeed
    N_45 = int(T_45 * 100 * k)
    traj.extend(mr.ScrewTrajectory(T4, T5, T_45, N_45, timeScaleing))
    gripper = np.concatenate([gripper, np.ones(N_45)])

    # standoff to final
    dist_56 = np.linalg.norm(T5[0:3,3] - T6[0:3,3])
    T_56 = dist_56 / standoffSpeed
    N_56 = int(T_01 * 100 * k)
    traj.extend(mr.ScrewTrajectory(T5, T6, T_56, N_56, timeScaleing))
    gripper = np.concatenate([gripper, np.ones(N_56)])

    # pause for release
    T_67 = gripDelay
    N_67 = int(T_67 * 100 * k)
    traj.extend(mr.ScrewTrajectory(T6, T7, T_67, N_67, timeScaleing))
    gripper = np.concatenate([gripper, np.zeros(N_67)])

    # release to standoff
    dist_78 = np.linalg.norm(T7[0:3,3] - T8[0:3,3])
    T_78 = dist_78 / standoffSpeed
    N_78 = int(T_78 * 100 * k)
    traj.extend(mr.ScrewTrajectory(T7, T8, T_78, N_78, timeScaleing))
    gripper = np.concatenate([gripper, np.zeros(N_78)])

    # return trajList
    return traj, gripper


def trajToList(traj, gripper):
    """Takes a list of Transformation matricies and a list of gripper states, 
    and returns them in the form r11, r12, r13, r21, r22, r23, r31, r32, r33,
    px, py, pz, gripper state
    """
    N = np.shape(traj)[0]
    configList = np.zeros((N,13))

    for i in range(0,N):
        configList[i,0] = traj[i][0][0] 
        configList[i,1] = traj[i][0][1] 
        configList[i,2] = traj[i][0][2] 
        configList[i,3] = traj[i][1][0] 
        configList[i,4] = traj[i][1][1] 
        configList[i,5] = traj[i][1][2] 
        configList[i,6] = traj[i][2][0] 
        configList[i,7] = traj[i][2][1] 
        configList[i,8] = traj[i][2][2] 
        configList[i,9] = traj[i][0][3] 
        configList[i,10] = traj[i][1][3] 
        configList[i,11] = traj[i][2][3] 
        configList[i,12] = gripper[i]

    return configList