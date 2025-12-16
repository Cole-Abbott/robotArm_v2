#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <SPIFFS.h>
#include <vector>


void readCSV(std::vector <Eigen::VectorXd> &trajectory)
{
    // SPIFFS should already be mounted before this call
    File file = SPIFFS.open("/trajectory.csv", "r");
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    while (file.available())
    {
        String line = file.readStringUntil('\n');
        Eigen::VectorXd pos = Eigen::VectorXd::Zero(3);
        sscanf(line.c_str(), "%lf,%lf,%lf", &pos(0), &pos(1), &pos(2));
        trajectory.push_back(pos);
    }

    file.close();
}