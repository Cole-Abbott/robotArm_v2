#include "kinematics.h"

// SECTION: Includes
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include "modern_robotics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// SECTION: Global Variables
MatrixXd Slist(6, 4);
MatrixXd Blist(6, 4);
MatrixXd M = MatrixXd::Identity(4, 4);

// SECTION: Function Definitions

// initialize global variables
void kinematicsSetup()
{
    // Define the joint screw axes in the space frame in mm
    Slist << 0, 0, 0, 0,
             0, -1, -1, 0,
             -1, 0, 0, 1,
             0, 85.9, 385.9, 0,
             0, 0, 0, 30,
             0, 0, 0, 0;

    // Define the home configuration (set M before computing Blist)
    M(0, 3) = 0;     // x
    M(1, 3) = 30;    // y
    M(2, 3) = 555.9; // z

    // Compute body screw axes from space screws and home transform
    Blist = mr::Adjoint(mr::TransInv(M)) * Slist;
}

/**
 * @brief Returns the end effector position given thetalist
 *
 * @param thetalist
 * @return MatrixXd
 */
MatrixXd forwardKinematics(VectorXd thetalist)
{
    // // Calculate the forward kinematics
    // MatrixXd T = mr::FKinBody(M, Blist, thetalist);
    MatrixXd T = mr::FKinSpace(M, Slist, thetalist);
    return T;
}

/**
 * @brief returns the thetalist given the end effector position, note this will not allways converge
 *
 * @param thetalist
 * @param T
 * @return VectorXd
 */
VectorXd inverseKinematics(VectorXd thetalist, MatrixXd T)
{
    // // Calculate the inverse kinematics
    bool converged = mr::IKinSpace(Slist, M, T, thetalist, 0.1, 0.01);
    // bool converged = mr::IKinBody(Blist, M, T, thetalist, 0.1, 0.01);
    if (!converged)
    {
        Serial.println("Inverse Kinematics did not converge");
    }
    return thetalist;
}

/**
 * @brief Returns the joint velocities required to move the end effector in the direction of the twist
 *
 * @param theta vector of joint angles
 * @param V 1x6 vector of the twist, angular velocity component is ignored
 * @return VectorXd thetadot 1x3 vector of joint velocities
 */
VectorXd moveInDirection(VectorXd theta, VectorXd V)
{
    // transform the twist to the end effector frame
    //  MatrixXd T_se = mr::FKinBody(M, Blist, theta);
    MatrixXd T_se = mr::FKinSpace(M, Slist, theta);
    MatrixXd T_se_inv = mr::TransInv(T_se);
    MatrixXd V_d = mr::Adjoint(T_se_inv) * V;

    // only interested in the linear components
    VectorXd V_d_lin = V_d.block(3, 0, 3, 1);

    // find the Jacobian
    MatrixXd Jb = mr::JacobianSpace(Slist, theta);

    // interested in the linear components
    int joints = theta.size();
    MatrixXd Jv = Jb.block(3, 0, 3, joints);
    // MatrixXd Jv = Jb.block(3, 0, 3, 4);

    // find the pseudo inverse of the Jv
    MatrixXd Jinv = Jv.completeOrthogonalDecomposition().pseudoInverse();

    return Jinv * V_d_lin;
}

/**
 * @brief Moves the end effector toward the desired position
 *
 * @param theta vector of joint angles
 * @param pos 1x3 vector of the desired end effector position in the s frame
 * @param Kp float the proportional gain
 * @return VectorXd thetalist 1x3 vector of joint angles
 */
VectorXd moveToPos(VectorXd theta, VectorXd pos, float Kp)
{
    // construct the desired end effector twist in the s frame
    MatrixXd T_se = forwardKinematics(theta); // current end effector position
    VectorXd actual_pos = T_se.block(0, 3, 3, 1);

    VectorXd V_lin = pos - actual_pos; // desired linear velocity (mm)
    // scale so magnitude is at most 1 mm/s
    float V_mag = V_lin.norm();
    if (V_mag > 1.0)
    {
        V_lin = (V_lin / V_mag) * 1.0;
    }
    // print out V_lin
    Serial.print("V_lin: ");
    for (int i = 0; i < V_lin.size(); i++)
    {
        Serial.print(actual_pos[i]);
        Serial.print(" ");
    }
    Serial.println();

    VectorXd V_ang = VectorXd::Zero(3); // no angular velocity
    VectorXd V = VectorXd::Zero(6);
    V.block(3, 0, 3, 1) = V_lin;
    V.block(0, 0, 3, 1) = V_ang;

    VectorXd dtheta = moveInDirection(theta, V); // calculate change in joint angles (radians)

    theta += Kp * dtheta; // update thetalist

    return theta;
}

/**
 * @brief Iterative IK that only cares about linear position (ignores orientation).
 *
 * @param Tsd Desired end-effector configuration (4x4 matrix)
 * @param thetalist Initial guess of joint angles (nx1 vector)
 * @param ev Position error tolerance
 * @param maxiter Maximum number of iterations
 * @return true if converged, false otherwise
 */
bool IKinBodyLinear(const MatrixXd &Tsd, VectorXd &thetalist, double ev, int maxiter, float step_scale)
{
    float lastErr = 1e6;
    VectorXd lastThetas = thetalist;
    for (int i = 0; i < maxiter; i++)
    {
        // Current end effector configuration
        MatrixXd Tsb = mr::FKinBody(M, Blist, thetalist);

        // Position error
        VectorXd p_sb = Tsb.block(0, 3, 3, 1);
        VectorXd p_sd = Tsd.block(0, 3, 3, 1);
        VectorXd p_err = p_sd - p_sb; // space-frame position error

        // Convergence check on linear error magnitude
        if (p_err.norm() < ev)
        {
            return true; // Converged
        } else if (p_err.norm() > lastErr) {
            step_scale = 0.1; // reduce step size if error increased
            thetalist = lastThetas; // revert to last thetas
        }
        // Serial.printf("Iter %d: Position error norm = %.6f \n", i, p_err.norm());

        // Jacobian
        MatrixXd Jb = mr::JacobianBody(Blist, thetalist);
        MatrixXd Jv = Jb.block(3, 0, 3, thetalist.size());

        // Pseudoinverse of Jv
        MatrixXd Jv_pinv = Jv.completeOrthogonalDecomposition().pseudoInverse();

        // Change in joint angles
        VectorXd dtheta = Jv_pinv * p_err;

        thetalist += dtheta * step_scale; // Dampen step for stability
        lastErr = p_err.norm();
        lastThetas = thetalist;
    }
    return false; // Did not converge within maxiter
}

// SECTION: End of File