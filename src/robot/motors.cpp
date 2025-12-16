// This file contains the implementation of the motor control functions
// By Cole Abbott, 2024

// SECTION: Includes
#include <Arduino.h>
#include "motors.h"
#include <ESP32Encoder.h>
#include "freertos/FreeRTOS.h"
#include "ArduinoEigenDense.h"
#include <AccelStepper.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// SECTION: DEFINES

#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define PULSE_WIDTH_RANGE (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 1048576
#define PWM_BIT_WIDTH 20
#define US_PER_STEP 20000

// SECTION: Global Variables
ESP32Encoder encoder_A;
ESP32Encoder encoder_B;

AccelStepper stepper_1(AccelStepper::DRIVER, STEPPER_1_STEP, STEPPER_1_DIR);
AccelStepper stepper_2(AccelStepper::DRIVER, STEPPER_2_STEP, STEPPER_2_DIR);
AccelStepper stepper_3(AccelStepper::DRIVER, STEPPER_3_STEP, STEPPER_3_DIR);


float servo_1_us = 500;
float servo_2_us = 500;

int setpointA = 0;
int setpointB = 0;


float jointLimits[4][2] = {{-PI, PI}, {-PI / 2, PI / 2}, {-PI / 2, PI / 2}, {-PI / 2, PI / 2}};
float jointZeros[4] = {0, 0, PI / 2 + 0.05 , PI / 2 + 0.02}; // Some joints are zeroed at a different angle
int jointReversals[4] = {1, -1, 1, 1};                      // Some joints are reversed

// SECTION: Function Prototypes
void motorControl(void *pvParameters);
void stepperRunTask(void *pvParameters);
void setServos(float angle_1, float angle_2);

// SECTION: Function Definitions

/**
    Sets up the motors for use
    Starts the motor control task
*/
void setupMotors()
{
    // Set the motor pins as outputs
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);

    // Setup the encoder
    encoder_A.attachHalfQuad(MOTOR_A_ENCODER_A, MOTOR_A_ENCODER_B);
    encoder_B.attachHalfQuad(MOTOR_B_ENCODER_A, MOTOR_B_ENCODER_B);

    // Setup ledc for servos
    ledcAttachPin(SERVO_PIN_1, 0);
    ledcAttachPin(SERVO_PIN_2, 1);
    ledcSetup(0, PWM_FREQUENCY, PWM_BIT_WIDTH);
    ledcSetup(1, PWM_FREQUENCY, PWM_BIT_WIDTH);

    // Setup steppers
    pinMode(STEPPER_1_STEP, OUTPUT);
    pinMode(STEPPER_1_DIR, OUTPUT);
    stepper_1.setMaxSpeed(5000);
    stepper_1.setAcceleration(3000);
    stepper_2.setMaxSpeed(5000);
    stepper_2.setAcceleration(3000);
    stepper_3.setMaxSpeed(5000);
    stepper_3.setAcceleration(3000);
     
    // Start the motor control task (DC motor PID & housekeeping)
    xTaskCreate(motorControl,    // The task function
                "Motor Control", // The task name
                10000,           // Stack size
                NULL,            // Task parameters
                1,               // Priority
                NULL);           // Task handle

    // Start a dedicated high-frequency task to service AccelStepper
    xTaskCreatePinnedToCore(
        stepperRunTask,      // Task function
        "Stepper Run",      // Name
        4096,                // Stack size
        NULL,                // Params
        1,                   // Priority (higher; runs more often)
        NULL,                // Handle
        1);                  // Pin to APP CPU (core 1)
}

/**
    @brief Sets servos to a given angles
    @param angle1 The angle to set the servo_1, angle in radians (0 to PI)
    @param angle2 The angle to set the servo_2, angle in radians (0 to PI)
*/
void setServos(float angle_1, float angle_2)
{

    // map without using the map function bc its for integers and angle is a float
    int us_1 = angle_1 / PI * PULSE_WIDTH_RANGE + MIN_PULSE_WIDTH;
    int us_2 = angle_2 / PI * PULSE_WIDTH_RANGE + MIN_PULSE_WIDTH;

    if (us_1 < MIN_PULSE_WIDTH) 
    {
        us_1 = MIN_PULSE_WIDTH;
    }
    if (us_1 > MAX_PULSE_WIDTH)
    {
        us_1 = MAX_PULSE_WIDTH;
    }
    if (us_2 < MIN_PULSE_WIDTH)
    {
        us_2 = MIN_PULSE_WIDTH;
    }
    if (us_2 > MAX_PULSE_WIDTH)
    {
        us_2 = MAX_PULSE_WIDTH;
    }
    
    int duty_1 = us_1 * PWM_RESOLUTION / US_PER_STEP;
    int duty_2 = us_2 * PWM_RESOLUTION / US_PER_STEP;



    ledcWrite(0, duty_1);
    ledcWrite(1, duty_2);
}

/**
    @brief Sets joints to the given angles
    @param angle1 The angle to set the joint 1 to, angle in radians
    @param angle2 The angle to set the joint 2 to, angle in radians
    @param angle3 The angle to set the joint 3 to, angle in radians
*/
void setJoints(VectorXd thetalist)
{
    float angle1 = thetalist(0);
    float angle2 = thetalist(1);
    float angle3 = thetalist(2);
    float angle4 = thetalist(3);

    // // apply joint limits
    if (angle1 < jointLimits[0][0])
    {
        angle1 = jointLimits[0][0];
    }
    if (angle1 > jointLimits[0][1])
    {
        angle1 = jointLimits[0][1];
    }
    if (angle2 < jointLimits[1][0])
    {
        angle2 = jointLimits[1][0];
    }
    if (angle2 > jointLimits[1][1])
    {
        angle2 = jointLimits[1][1];
    }
    if (angle3 < jointLimits[2][0])
    {
        angle3 = jointLimits[2][0];
    }
    if (angle3 > jointLimits[2][1])
    {
        angle3 = jointLimits[2][1];
    }
    if (angle4 < jointLimits[3][0])
    {
        angle4 = jointLimits[3][0];
    }
    if (angle4 > jointLimits[3][1])
    {
        angle4 = jointLimits[3][1];
    }

    // // after joint limits, save thetalist to return
    // VectorXd newTheta = VectorXd::Zero(4);
    // newTheta(0) = angle1;
    // newTheta(1) = angle2;
    // newTheta(2) = angle3;
    // newTheta(3) = angle4;
    setJoint1(angle1);
    setJoint2(angle2);
    setJoint3(angle3);
    setJoint4(angle4);

    // setServos(angle3, angle4);

    // // convert radians to encoder ticks
    // int setpoint_1 = angle1 / (2 * PI) * 16 * 48 * 4 * 2.45; //  4:1 gear ratio, 16 ticks per encoder, 48:1 gear ratio, 2.45:1 mystery correction factor

    // int setpoint_2 = angle2 / (2 * PI) * 408 * 2 * 2; //  2:1 gear ratio, 408 ticks per encoder * 2 because of half quad

    // setpointA = setpoint_1;
    // setpointB = setpoint_2;

    // return newTheta;
}

/**
 * @brief Sets motors to a given speeds
 * @param speed_a The speed to set motor A to
 * @param speed_b The speed to set motor B to
 */
void setMotors(int speed_a, int speed_b)
{
    if (speed_a > 0)
    {
        digitalWrite(MOTOR_A_DIR, LOW);
        analogWrite(MOTOR_A_PWM, speed_a);
    }
    else
    {
        digitalWrite(MOTOR_A_DIR, HIGH);
        analogWrite(MOTOR_A_PWM, -speed_a);
    }

    if (speed_b > 0)
    {
        if (speed_b > 250) speed_b = 250; // limit max speed
        digitalWrite(MOTOR_B_DIR, LOW);
        analogWrite(MOTOR_B_PWM, speed_b);
    }
    else
    {
        if (speed_b < -250) speed_b = -250; // limit max speed
        digitalWrite(MOTOR_B_DIR, HIGH);
        analogWrite(MOTOR_B_PWM, -speed_b);
    }
}



void motorControl(void *pvParameters)
{
    Serial.println("Motor Control Task Started");
    int lastError_A = 0;
    int lastError_B = 0;

    while (1)
    {
        // Motor A
        int encoderValue_A = encoder_A.getCount();
        int error_A = setpointA - encoderValue_A;

        int derivative_A = error_A - lastError_A;
        lastError_A = error_A;

        int pwm_A = KP_A * error_A + KD_A * derivative_A;

        // Motor B
        int encoderValue_B = encoder_B.getCount();
        int error_B = setpointB - encoderValue_B;
        
        int derivative_B = error_B - lastError_B;
        lastError_B = error_B;

        int pwm_B = KP_B * error_B + KD_B * derivative_B;

        setMotors(pwm_A, pwm_B);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


// Sets joint 1 to a given angle in radians
void setJoint1(float angle)
{
    int steps = angle / (2 * PI) * JOINT_1_STEPS_PER_REV;
    stepper_1.moveTo(steps);
}

// Sets joint 2 to a given angle in radians
void setJoint2(float angle)
{
    int steps = angle / (2 * PI) * JOINT_2_STEPS_PER_REV;
    stepper_2.moveTo(steps);
}

// Sets joint 3 to a given angle in radians
void setJoint3(float angle)
{
    int steps = angle / (2 * PI) * JOINT_3_STEPS_PER_REV;
    stepper_3.moveTo(-steps);
}

void setJoint4(float angle)
{
    setpointB = angle / (2 * PI) * JOINT_4_TICKS_PER_REV; // 4:1 gear ratio, 16 ticks per encoder, 48:1 gear ratio, 2.45:1 mystery correction factor
}

void zeroJoints(VectorXd &thetalist) {
    // Zero joint 1
    stepper_1.setCurrentPosition(0);
    thetalist(0) = 0.0;

    // Zero joint 2
    stepper_2.setCurrentPosition(0);
    thetalist(1) = 0.0;

    // Zero joint 3
    stepper_3.setCurrentPosition(0);
    thetalist(2) = 0.0;

    // Zero joint 4
    encoder_B.setCount(0);
    thetalist(3) = 0.0;

    setJoints(thetalist);
}

// High-frequency task that calls stepper_1.run() as often as practical
void stepperRunTask(void *pvParameters)
{
    for (;;)
    {

        stepper_1.run();
        stepper_2.run();
        stepper_3.run();
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


// END OF FILE