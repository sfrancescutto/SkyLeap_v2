#include "mbed.h"
#include <cstdint>

#ifndef MPU9150_H
#define MPU9150_H

//Accelerometer range selector
enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
    };

//Gyroscope range selector
enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
    };

void writeByte(uint8_t address, uint8_t regster, uint8_t data);
char readByte(uint8_t address, uint8_t regster);
void readBytes(uint8_t address, uint8_t regster, uint8_t count, uint8_t * dest);  
void initAK8975A(float * destination);
void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void readMagData(int16_t * destination);
void initMPU9150();

#endif
