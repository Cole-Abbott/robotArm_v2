#include <Arduino.h>



// #define mySSID "X6Nt8.2"
// #define PASSWORD "frisBee520crepes"
#define mySSID "Fios-X6Nt8"
#define PASSWORD "pig44hives75gnp"
// #define mySSID "Cole's iPhone"
// #define PASSWORD "Keiko1818"


void start_web_services(void (*callback)(float, float, float), void (*jointCallback)(float, float, float, float), void (*zeroCallback)());
uint8_t startSPIFFS();

// Enum for message types
enum MessageType
{
    POSITION,
    JOINTS,
    ZERO
};