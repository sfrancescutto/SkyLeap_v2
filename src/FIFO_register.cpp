#include "FIFO_register.h"
#include "mbed.h"
#include <cstdint>

// this class implement a FIFO Register with in output the average of the array
// the length of the register is defined in the constructor by "int len"

FifoReg::FifoReg(int len) {
    this->length=len;
    regist = new int[length];
}

// shift the register: discard the oldest value and return the oldest value
int FifoReg::FifoReg_shift(int new_val) {
    int output = regist[length-1];
    for (int i=length-1; i>1; i--) {
        regist[i] = regist[i-1];
    }
    regist[0] = new_val;
    return output;
}

// shift the register: discard the oldest value and return the average
int FifoReg::FifoReg_shift_and_m_av(int new_val) {
    volatile int average=0;
    for (int i=length-1; i>=1; i--) {
        regist[i] = regist[i-1];
        average += regist[i];
    }
    regist[0] = new_val;
    average += regist[0];
    average /= length;
    return average;
}

//print the register
void FifoReg::FifoReg_print() {
    for (int i=length-1; i>=0; i--) {
        printf("%6d ",regist[i]);
    }
    printf("\n ");
}
