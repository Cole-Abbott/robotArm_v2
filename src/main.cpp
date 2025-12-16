#include <Arduino.h>
#include <vector>
#include <ArduinoEigenDense.h>
#include "robot/modern_robotics.h"

#include "robot/motors.h"
#include "robot/kinematics.h"
#include "robot/trajectory.h"
#include "server/myServer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Import required libraries
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd thetalist(4);
std::vector<VectorXd> trajectory;

struct PositionCommand
{
  float x;
  float y;
  float z;
};

PositionCommand initialPos;
QueueHandle_t positionQueue = nullptr;

void controlTask(void *pvParameters);

void posCallback(float x, float y, float z)
{
  if (!positionQueue)
  {
    return; // queue not ready yet
  }

  PositionCommand cmd{x, y, z};
  xQueueOverwrite(positionQueue, &cmd); // keep only the latest target
}

void jointCallback(float t1, float t2, float t3, float t4)
{
  thetalist(0) = t1;
  thetalist(1) = t2;
  thetalist(2) = t3;
  thetalist(3) = t4;
  setJoints(thetalist);
}

void zeroCallback()
{
  zeroJoints(thetalist);
}


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  setupMotors();
  Serial.println("Motor Setup Complete");

  kinematicsSetup();
  thetalist << 0, 0, 0, 0; // assume inital joint angles are all 0
  // thetalist << 0, -PI / 3, PI / 2, 0; // initial joint angles

  MatrixXd pos = forwardKinematics(thetalist);
  initialPos.x = pos(0, 3);
  initialPos.y = pos(1, 3);
  initialPos.z = pos(2, 3);

  setJoints(thetalist);

  Serial.println("Kinematics Setup Complete");

  // readCSV(trajectory);
  // Serial.println("Trajectory Read Complete");
  // Serial.printf("Trajectory size: %d\n", trajectory.size());

  // Initialize SPIFFS
  uint8_t spiffs_check = startSPIFFS();
  if (spiffs_check == 1)
  {
    Serial.println("SPIFFS ERROR!");
    return;
  }

  // Queue for latest position command (single-slot overwrite to avoid backlog)
  positionQueue = xQueueCreate(1, sizeof(PositionCommand));
  if (!positionQueue)
  {
    Serial.println("Failed to create position queue");
    return;
  }

  // Task to run IK and send joint targets at a controlled rate
  xTaskCreatePinnedToCore(
      controlTask,
      "ControlTask",
      8192,
      NULL,
      1,
      NULL,
      0);

  start_web_services(posCallback, jointCallback, zeroCallback);

}



void loop()
{  
  // // make robot move in a loop around initial position
  // static unsigned long lastMoveTime = 0;
  // const unsigned long moveInterval = 100; // milliseconds 
  // if (millis() - lastMoveTime >= moveInterval) {
  //   lastMoveTime = millis();

  //   static float angle = 0.0;
  //   angle += 0.05; // increment angle

  //   PositionCommand cmd;
  //   cmd.x = initialPos.x + 25 * cos(angle) - 25; // 50 mm radius circle
  //   cmd.y = initialPos.y + 25 * sin(angle);
  //   cmd.z = initialPos.z; // keep z constant

  //   setJoint4(sin(angle)); // rotate joint 4 continuously

  //   // posCallback(cmd.x, cmd.y, cmd.z);
  // }
}

// Runs IK on the latest target and sends joint goals without blocking the WebSocket thread
void controlTask(void *pvParameters)
{
  PositionCommand latest{};
  bool hasTarget = false;

  for (;;)
  {
    // Grab most recent target if available
    if (xQueueReceive(positionQueue, &latest, pdMS_TO_TICKS(5)) == pdTRUE)
    {
      hasTarget = true;
    }

    if (!hasTarget) // no target yet
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    MatrixXd Tsd = MatrixXd::Identity(4, 4);
    Tsd(0, 3) = latest.x;
    Tsd(1, 3) = latest.y;
    Tsd(2, 3) = latest.z;

    MatrixXd pos = forwardKinematics(thetalist);
    Serial.printf("Current Pos: %.2f, %.2f, %.2f | ", pos(0, 3), pos(1, 3), pos(2, 3));
    Serial.printf("Target Pos: %.2f, %.2f, %.2f\n", latest.x, latest.y, latest.z);

    bool converged = IKinBodyLinear(Tsd, thetalist, 1e-2, 40, 1);
    if (!converged)
    {
      // avoid spamming serial, only note when failing
      Serial.println("IK did not converge for target");
    }

    setJoints(thetalist);
    Serial.printf(">t1: %f\n", thetalist(0));
    Serial.printf(">t2: %f\n", thetalist(1));
    Serial.printf(">t3: %f\n", thetalist(2));
    Serial.printf(">3D|mySimpleCube:S:sphere:P:%f:%f:%f:RA:0.25:C:#2ecc71\n", latest.x/100, latest.y/100, latest.z/100);

    vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz update to keep stepper task breathing room
  }
}