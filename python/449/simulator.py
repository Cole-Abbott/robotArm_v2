# simulator.py 
# by Cole Abbott
# ME-449 Final Project Milestone 1

import numpy as np
import modern_robotics as mr


def nextState(q, qdot, dt, maxSpeed):
    """Finds the next state of the youBot

    :param q: 12 Vector representing the configuration of the robot: 
    q = [phi,x,y]U[theta]U[wheel angles]. shape = (3,5,4)

    :param qdot: 9 vector of speeds: qdot = [u]U[thetadot] (4,5)

    :param dt: timestep

    :param maxSpeed: Max joint speeds for wheels and arm joints

    :return: 12 Vector representing the configuration of the robot time dt later
    """

    # limit velocities to maxSpeed
    for i in range(len(qdot)):
        if qdot[i] > maxSpeed:
            qdot[i] = maxSpeed
        if qdot[i] < -maxSpeed:
            qdot[i] = -maxSpeed

    # parse the inputs
    theta = np.array(q[3:8])
    thetadot = np.array(qdot[4:9])
    chasisPos = np.array(q[0:3]) # [x,y,phi]
    wheelPos = np.array(q[8:12])
    u = np.array(qdot[0:4]) # u = 4 vector of wheel velocities

    # simulate arm and wheel movement with Euler's method
    theta = theta + dt * thetadot
    wheelPos = wheelPos + dt * u

    # simulate chasis movement with odometry
    # find chasisVel from u: chasisVel = Fu
    l,w,r = 0.47/2,0.3/2,0.0475 # length, width, and wheel radius of the robot

    F = r / 4 * np.array([
        [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
        [1,1,1,1],
        [-1,1,-1,1]
    ])

    chasisVel = F @ u # [phidot, xdot, ydot] in the robot base frame
    
    # use Euler's method to update robot position
    phi = chasisPos[0] + dt * chasisVel[0]
    dx = dt * chasisVel[1]
    dy = dt * chasisVel[2]
    x = chasisPos[1] + dx * np.cos(phi) - dy * np.sin(phi)
    y = chasisPos[2] + dy * np.cos(phi) + dx * np.sin(phi)

    # format and return updated position
    chasisPosArray = np.array([phi,x,y])

    return np.concatenate([chasisPosArray, theta, wheelPos])


def endEffectorPos(q, M, Blist):
    """Takes the youBot configuration and returns the transformation T_se

    :param q: 12 Vector representing the configuration of the robot: 
    :param M: Home configuration of the robot arm
    :param Blist: screw axis of the robot arm

    q = [phi,x,y]U[theta]U[wheel angles]. shape = (3,5,4)
    """

    # find T_sb: {s} = space frame, {b} = robot chasis frame
    phi = q[0]
    R_sb = np.array([[np.cos(phi), -np.sin(phi), 0],[np.sin(phi), np.cos(phi), 0],[0,0,1] ])
    T_sb = mr.RpToTrans(R_sb,np.array([q[1], q[2], 0.0963]))


    # find T_b0: {0} = robot arm base frame
    R_b0 = np.eye(3)
    T_b0 = mr.RpToTrans(R_b0,np.array([0.1662, 0, 0.0026]))


    # find T_0e: {e} = end effector frame
    thetalist = q[3:8]
    T_0e = mr.FKinBody(M,Blist,thetalist)

    return T_sb @ T_b0 @ T_0e



def Jacobian(q,M,Blist):
    """Finds the body Jacobian of the youBot

    :param q: 12 Vector representing the configuration of the robot: 
    :param M: Home configuration of the robot arm
    :param Blist: screw axis of the robot arm

    q = [phi,x,y]U[theta]U[wheel angles]. shape = (3,5,4)

    """
    thetalist = q[3:8]
    J_arm = mr.JacobianBody(Blist, thetalist)

    l,w,r = 0.47/2,0.3/2,0.0475 # length, width, and wheel radius of the robot
    F = r / 4 * np.array([
        [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
        [1,1,1,1],
        [-1,1,-1,1]
    ])

    # make the F6 matrix
    F6 = np.concatenate([np.zeros((2,4)), F, np.zeros((1,4))])

     # find T_b0: {0} = robot arm base frame
    R_b0 = np.eye(3)
    T_b0 = mr.RpToTrans(R_b0,np.array([0.1662, 0, 0.0026]))


    # find T_0e: {e} = end effector frame
    thetalist = q[3:8]
    T_0e = mr.FKinBody(M,Blist,thetalist)

    # Adjoint transformatiion from {e} to {b}
    Ad_eb = mr.Adjoint(mr.TransInv(T_b0 @ T_0e))

    J_base = Ad_eb @ F6

    J = np.concatenate([J_base.T, J_arm.T]).T

    return J


def testJointLimits(q, qdot, dt, maxSpeed, limits):
    """
    Tests if the next state of the youBot will violate the joint limits
    param limits: A list of tuples representing the joint limits of the youBot arm
    return: True if the next state violates the joint limits
     """

    # find the next state of the youBot
    newQ = nextState(q,qdot,dt,maxSpeed) 

    violations = np.zeros(5)

    # check if the new state violates the joint limits,
    #  and velocity is moving towards more violation
    for i in range(5):
        if newQ[i+3] > limits[i][0]:
            violations[i] = 1
        if newQ[i+3] < limits[i][1]:
            violations[i] = 1

    return violations


