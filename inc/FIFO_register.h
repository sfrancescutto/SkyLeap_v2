#include "mbed.h"
#include <cstdint>

#ifndef FIFO_REGISTER_H
#define FIFO_REGISTER_H

// this class implement a FIFO Register with in output the average of the array
// the length of the register is defined in the constructor by "int len"

class FifoReg{
    private:
        int length;
        int* regist;

    public:
        FifoReg(int len);
        int FifoReg_shift(int new_val);
        int FifoReg_shift_and_m_av(int new_val);
        void FifoReg_print();
};

#endif
