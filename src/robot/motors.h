// Header file for motors.cpp
// Contains the pin definitions for the motors and servos
// Also contains the function prototypes for the motors.cpp file
// By Cole Abbott, 2024

// SECTION: Includes
#include <Arduino.h>
#include <ArduinoEigenDense.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// SECTION: Pin Definitions
#define SERVO_PIN_1 32 // Servo 1 is Joint 2
#define SERVO_PIN_2 14 // Servo 2 is Joint 3

#define MOTOR_B_PWM 33
#define MOTOR_B_DIR 25
#define MOTOR_A_PWM 26 // Motor A is Joint 1
#define MOTOR_A_DIR 27
#define KP_A 15
#define KD_A 40
#define KP_B 15
#define KD_B 40

#define MOTOR_A_ENCODER_A 22
#define MOTOR_A_ENCODER_B 23

#define MOTOR_B_ENCODER_A 21
#define MOTOR_B_ENCODER_B 19
#define JOINT_4_TICKS_PER_REV  16 * 48 * 6 // 4:1 gear ratio, 16 ticks per encoder, 48:1 gear ratio, 2.45:1 mystery correction factor

#define STEPPER_1_STEP 17
#define STEPPER_1_DIR 5

#define STEPPER_2_STEP 16
#define STEPPER_2_DIR 18

#define STEPPER_3_STEP 2
#define STEPPER_3_DIR 4

#define JOINT_1_STEPS_PER_REV 32000 // 200 * 16 * 10 200 steps per rev 16x microstepping, 10x gear ratio
#define JOINT_2_STEPS_PER_REV 69688.8888889 // 200 * 16 * 4.666 * 4.666, 200 steps per rev 16x microstepping, 2x 4.66 planetary gearboxes
#define JOINT_3_STEPS_PER_REV 32000 // 38 * 16 * 5, 48 steps per rev 16x microstepping, 25/3 internal gear ratio, 5:1 external gear ratio

// #define JOINTLIMITS  {{ -2 * PI, 2 * PI }, { 0, 2 }, { PI / 2, PI } }// Joint limits for the robot


// SECTION: Function Prototypes
void setupMotors();
void setJoint1(float angle);
void setJoint2(float angle);
void setJoint3(float angle);
void setJoint4(float angle);
void setJoint5(float angle);
void setJoint6(float angle);


void setJoints(VectorXd thetalist);
void zeroJoints(VectorXd &thetalist);
bool reachedSetpoints();