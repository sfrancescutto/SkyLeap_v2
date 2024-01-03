#include "mbed.h"
#include <cstdint>

#ifndef CHANNEL_H
#define CHANNEL_H

/*
this class take care of radio reciever input
The signal comes as a PWM of 50 Hz frequency and a duty cycle of 5to10% 
an interrupt signal the rise and fall of the pwm wave to a timer. 
The interval is corrected and remapped in the range [0,1000]us.
*/
class Channel{
private:
    InterruptIn interrupt;
    Timer timer;
    volatile int time;
    int number;     //name id 
    int max, min;   //time boundaries
    int offset;     
    int new_max;
    float factor;   
    int calibrated; //corrected values
    int last_values[3];
    short i;

public:
    Channel(PinName pin, int num);

    void start();       //IRS: start the timer    
    void stop();    
    void find_interval();
    void calculate_calibration();
    void channel_default_value(int default_offset, float default_factor);

    /*
        read operation require the application of offset and scaling factor:
        it's possible to calibrate one time and read separately
        or doing all toghether one time 
    */
    void calibrate();
    //int calibrate_read();
    int read();

    // ----------- PRINT METHODS ---------------------
    void get_parameters();
    void print();
};

#endif
