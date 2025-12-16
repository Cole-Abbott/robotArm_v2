import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def deg_to_rad(degrees):
    return degrees * np.pi / 180.0

def rad_to_deg(radians):
    return radians * 180.0 / np.pi


Slist = np.array([[0, 0, 0],
                  [0, -1, -1],
                  [-1, 0, 0],
                  [0, 85.9, 385.9],
                  [0, 0, 0],
                  [0, 0, 0]])
M = np.array([[1, 0, 0, 0],
              [0, 1, 0, 30],
              [0, 0, 1, 555.9],
              [0, 0, 0, 1]])
thetalist = np.array([0, 0, 0])

Blist = mr.Adjoint(mr.TransInv(M)) @ Slist
print("Blist:   \n", Blist)

def ForwardKinematics(thetalist):
        T = mr.FKinSpace(M, Slist, thetalist)
        return T

def IKinBodyLinear(Tsd, thetalist0, ev=1e-2, maxiter=40, step_size=1):
    """Iterative IK that only cares about linear position (ignores orientation).

    Args:
        Tsd: 4x4 desired pose (only translation is used).
        thetalist0: initial guess (n,).
        eomg: unused (kept for API compatibility).
        ev: linear error tolerance (magnitude of position error in the space frame).
        maxiter: max iterations.

    Returns:
        (thetalist, success, iters)
    """
    err_arr = []
    dtheta_arr = []
    thetalist_arr = []
    pos_arr = []

    dxyz_arr = []
    thetalist = thetalist0.astype(float).copy()
    thetalist_arr.append(thetalist.copy())
    current_error_norm = float('inf')
    last_error_norm = float('inf')
    last_dtheta = np.zeros_like(thetalist)
    last_last_dtheta = np.zeros_like(thetalist)
    for i in range(maxiter):
        T_sb = mr.FKinBody(M, Blist, thetalist)
        p_sb = T_sb[0:3, 3]
        p_sd = Tsd[0:3, 3]
        p_err = p_sd - p_sb  # space-frame position error
        print(f"Iteration {i}: position error = {p_err}, norm = {np.linalg.norm(p_err)}")

        new_error_norm = np.linalg.norm(p_err)
    
        # --- Adaptive Damping Logic ---
        if i > 0 and new_error_norm > current_error_norm and new_error_norm > last_error_norm and current_error_norm > last_error_norm: # error increased for two iterations
            print(f"Iteration {i}: Error increased from {current_error_norm} to {new_error_norm}. Reducing step size.")
           
            # reverse direction of last step
            thetalist = thetalist - last_dtheta * step_size - last_last_dtheta * step_size

            # add some random noise to escape local minima
            noise = np.random.uniform(-0.01, 0.01, size=thetalist.shape)
            thetalist += noise
            # recompute error after reverting
            T_sb = mr.FKinBody(M, Blist, thetalist)
            p_sb = T_sb[0:3, 3]
            p_err = p_sd - p_sb  # space-frame position error
            new_error_norm = np.linalg.norm(p_err)
            step_size *= -1  # Reduce step size if error increased
            print(f"Iteration {i}: position error = {p_err}, norm = {np.linalg.norm(p_err)}")

        pos_arr.append(p_sb.copy())
        err_arr.append(p_err.copy())
        
        last_error_norm = current_error_norm
        current_error_norm = new_error_norm

        # Convergence check on linear error magnitude
        if new_error_norm < ev:
            err_arr = np.array(err_arr)
            plt.figure()
            plt.plot(np.linalg.norm(err_arr, axis=1))
            plt.show()
            plt.plot(thetalist_arr)
            plt.show()
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(err_arr[:,0], err_arr[:,1], err_arr[:,2], marker='o')
            ax.plot(err_arr[0,0], err_arr[0,1], err_arr[0,2], marker='x', color='green', markersize=10, label='Start')
            ax.plot([0], [0], [0], marker='x', color='red', markersize=10, label='Origin')

            ax.set_xlabel('X Error (m)')
            ax.set_ylabel('Y Error (m)')
            ax.set_zlabel('Z Error (m)')
            ax.set_title('End-Effector Position Error Trajectory')
            ax.legend()
            plt.show()

            return thetalist, True, i

        J = mr.JacobianBody(Blist, thetalist)  # 6xn
        Jv = J[3:, :]  # linear part (3xn)

        # Compute pseudo-inverse of Jv for least-squares step
        Jv_pinv = np.linalg.pinv(Jv)
        dtheta = Jv_pinv @ p_err

        dtheta_arr.append(dtheta)

        thetalist = thetalist + dtheta * step_size  # Dampen step for stability
        
        last_last_dtheta = last_dtheta
        last_dtheta = dtheta
        thetalist_arr.append(thetalist.copy())

        # actual change in position
        T_sb_new = mr.FKinBody(M, Blist, thetalist)
        dxyz_arr.append(-(p_sb[0] - T_sb_new[0, 3]))

    err_arr = np.array(err_arr)
    plt.figure()
    plt.plot(np.linalg.norm(err_arr, axis=1))
    plt.show()
    plt.plot(thetalist_arr)
    plt.show()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(err_arr[:,0], err_arr[:,1], err_arr[:,2], marker='o')
    ax.plot(err_arr[0,0], err_arr[0,1], err_arr[0,2], marker='x', color='green', markersize=10, label='Start')
    ax.plot([0], [0], [0], marker='x', color='red', markersize=10, label='Origin')

    ax.set_xlabel('X Error (m)')
    ax.set_ylabel('Y Error (m)')
    ax.set_zlabel('Z Error (m)')
    ax.set_title('End-Effector Position Error Trajectory')
    ax.legend()
    plt.show()
    return thetalist, False, maxiter

thetalist = np.array([0, -np.pi/3, -np.pi/2]) 
pos = ForwardKinematics(thetalist)

Tsd = np.eye(4)

Tsd[0,3] = pos[0,3] + 25 * np.cos(0.3) - 25
Tsd[1,3] = pos[1,3] + 25 * np.sin(0.3) + 50
Tsd[2,3] = pos[2,3] + 32
print(f"x{Tsd[0,3]}, y{Tsd[1,3]}, z{Tsd[2,3]}")

thetalist,converged, _ = IKinBodyLinear(Tsd, thetalist)
print(converged)