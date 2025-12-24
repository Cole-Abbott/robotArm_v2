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


int setpointA = 0;
int setpointB = 0;


float jointLimits[4][2] = {{-PI, PI}, {-PI / 2, PI / 2}, {-PI / 2, PI / 2}, {-PI, PI}};
float jointZeros[4] = {0, 0, PI / 2 + 0.05 , PI / 2 + 0.02}; // Some joints are zeroed at a different angle
int jointReversals[4] = {1, -1, 1, 1};                      // Some joints are reversed

// SECTION: Function Prototypes
void motorControl(void *pvParameters);
void stepperRunTask(void *pvParameters);

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
    stepper_3.setMaxSpeed(2000);
    stepper_3.setAcceleration(1000);
     
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
    @brief applies joint angles and ets joints to the given angles
    @param thetalist A VectorXd of joint angles in radians
*/
void setJoints(VectorXd thetalist)
{
    float angle1 = thetalist(0);
    float angle2 = thetalist(1);
    float angle3 = thetalist(2);
    float angle4 = thetalist(3);
    float angle5 = thetalist(4);
    float angle6 = thetalist(5);

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

    setJoint1(angle1);
    setJoint2(angle2);
    setJoint3(angle3);
    setJoint4(angle4);
    setJoint5(angle5);
    setJoint6(angle6);
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

void setJoint5(float angle) {
    angle += PI / 2; // adjust for servo mounting
    // map without using the map function bc its for integers and angle is a float
    int us = angle / PI * PULSE_WIDTH_RANGE + MIN_PULSE_WIDTH;
    if (us < MIN_PULSE_WIDTH) 
    {
        us = MIN_PULSE_WIDTH;
    }
    if (us > MAX_PULSE_WIDTH)
    {
        us = MAX_PULSE_WIDTH;
    }
    int duty = us * PWM_RESOLUTION / US_PER_STEP;
    ledcWrite(1, duty);
}


void setJoint6(float angle) {
    angle += PI / 2 + 0.1; // adjust for servo mounting angle
    // map without using the map function bc its for integers and angle is a float
    int us = angle / PI * PULSE_WIDTH_RANGE + MIN_PULSE_WIDTH;
    if (us < MIN_PULSE_WIDTH) 
    {
        us = MIN_PULSE_WIDTH;
    }
    if (us > MAX_PULSE_WIDTH)
    {
        us = MAX_PULSE_WIDTH;
    }
    int duty = us * PWM_RESOLUTION / US_PER_STEP;
    ledcWrite(0, duty);
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

bool reachedSetpoints() {
    bool joint1Reached = (stepper_1.distanceToGo() == 0);
    bool joint2Reached = (stepper_2.distanceToGo() == 0);
    bool joint3Reached = (stepper_3.distanceToGo() == 0);
    bool joint4Reached = (abs(setpointB - encoder_B.getCount()) < 20); // within 5 ticks

    return joint1Reached && joint2Reached && joint3Reached && joint4Reached;
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