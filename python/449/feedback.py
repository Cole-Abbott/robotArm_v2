# feedback.py
# by Cole Abbott
# ME-449 Final Project Milestone 3

import numpy as np
import modern_robotics as mr

def FeedbackControl(X, X_d, X_d_next, X_err_int, gains, dt):
    """Calculates the desired end effector twist V such that 
    the end effector will follow the trajectory

    :param X: Actual end-effector configuration

    :param X_d: The desired end-effector configuration

    :param X_d_next: The desired end-effector configuration for the next timestep

    :param X_int: The integral error of the end-effector

    :param gains: The PI gains [K_p, K_i]

    :param dt: The timestep between reference trajectory configurations

    :return: The desired end effector twist V, and X_err for plotting
    """

    Kp, Ki = gains
    
    # error twist that takes X to X_d in unit time
    X_err = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X) @ X_d))

    # add curent error to integral error twist
    X_err_int = X_err_int + dt * X_err

    # feedforward twist from X_d to X_d_next in time dt
    V_d = mr.se3ToVec(1/dt * mr.MatrixLog6(mr.TransInv(X_d) @ X_d_next))
    

    # transformation from desired to actual end-effector frame, 
    # because V_d is in desired end effector frame
    Ad = mr.Adjoint(mr.TransInv(X) @ X_d)

    # desired end effector twist
    V = Ad @ V_d + Kp * X_err + Ki * X_err_int
    # V = Kp * X_err + Ki * X_err_int

    return V, X_err, X_err_int



