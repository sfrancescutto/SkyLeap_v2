#include <cstdint>
#include "mbed.h"
#include "mbed.h"
#include "math.h"
#include <cstdint>
#include <cstdio>
#include "MPU9150.h"
//#include "Channel.h"  //commented because already included in calibration
#include "Calibration.h"
#include "Lights.h"
#include "FIFO_register.h"
#include "Quaternion.hpp"

#ifndef SKYLEAP_UTILS
#define SKYLEAP_UTILS
void initialize();
void select_mode();
void readSensors(float& pitch, float& roll, float& yaw);
void readRC(int& pitch_setpoint, int& roll_setpoint, int& yawspeed_setpoint, int& throttle);
void produceESCOutput(const float& pitch, const float& roll, const float& yaw, const int& pitch_setpoint, const int& roll_setpoint, const int& yawspeed_setpoint, const int& throttle);
void provaRadiocomando();
void provaMotori();
void provaSensori();
void main_loop();
#endif
