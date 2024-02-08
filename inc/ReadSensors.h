#include <cstdint>
#include "mbed.h"
#ifndef READSENSORS_H
#define READSENSORS_H
    void readSensors(int offset_acc_main[], int offset_gyr_main[], Timer CycleTimer, double& totalrollf, double& totalpitchf, double& totalyawf);
#endif
