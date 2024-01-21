#include <cstdint>
#include "mbed.h"
#include "Channel.h"
#ifndef CALIBRATION_H
#define CALIBRATION_H
  void readAccelData(std::int16_t *destination);    // calibration need to read mpu data 
  void readGyroData(std::int16_t * destination);    // import some header MPU9150 functions
  void readMagData(int16_t *destination);
  
  bool acc_gyr_calibration(int sample, int* acc_offset, int* gyr_offset);
  void mag_calibration(int* hard_mag, float* soft_mag); //TODO Ho cambiato da bool a void
  void default_mag();
  void readMagData_calibr(int16_t * destination, int* hard_mag, float* soft_mag);
  float cutOff(float value, float min, float max);
  int cutOffInt(int value, int min, int max);
#endif
