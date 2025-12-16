
// SECTION: Includes
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include "modern_robotics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// SECTION: Global Variables



// SECTION: Function Definitions

void kinematicsSetup();
MatrixXd forwardKinematics(VectorXd thetalist);

VectorXd inverseKinematics(VectorXd thetalist, MatrixXd T);
VectorXd moveInDirection(VectorXd thetalist, VectorXd twist);
VectorXd moveToPos(VectorXd thetalist, VectorXd pos, float Kp);
bool IKinBodyLinear(const MatrixXd& Tsd, VectorXd& thetalist, double ev=1e-2, int maxiter=40, float step_scale=1.0);