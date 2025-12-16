#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <SPIFFS.h>
#include <vector>


void readCSV(std::vector <Eigen::VectorXd> &trajectory);