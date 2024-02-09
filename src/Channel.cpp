#include "Channel.h"
#include "mbed.h"
#include <cstdint>
#include <Callback.h>

Channel::Channel(PinName pin, int num) : interrupt(pin) {    //constructor
    this->number=num;
    //interrupt.rise(&Channel::start);
    //interrupt.fall(&Channel::stop);
    interrupt.rise([this](){start();}); //I use lambda functions as KSC doesn't seem to like the previous expression
    interrupt.fall([this](){stop();});

    offset=1000; min=1000; max=0;
    int last_values[3] = {0, 0, 1000};
    short i = 0;
    factor=1;
}

void Channel::start() {
    timer.start();
    }        //IRS: start the timer

void Channel::stop() {                           //IRS: stop the timer
    timer.stop();
    // time  = timer.read_us()-1000;       //return time -1000
    time = timer.elapsed_time().count() - 1000;
    timer.reset();                      //reset timer
}

void Channel::reset_timer() {
    //timer.reset();
}

void Channel::find_interval(){     //this function verify if value exceed border
    if (time > max){               //it must be called several times during calibration         
        max = time; 
    } else if (time < min){     
        min = time; 
    }
}
void Channel::calculate_calibration(){                   //this function utilze the updated interval
    offset = min+50;                            //it must be called one time after find_interval() per cycle
    new_max=max-offset;
    factor = 1100.00000/new_max;
}

void Channel::channel_default_value(int default_offset, float default_factor){
    offset = default_offset;
    factor = default_factor;
}

/*
    read operation require the application of offset and scaling factor:
    it's possible to calibrate one time and read separately
    or doing all toghether one time 
*/

void Channel::calibrate() {                              
    calibrated = (time-offset)*factor;
    if (calibrated>1000) {
        calibrated=1000;
        } else 
        if (calibrated<0) {
        calibrated=0;
        } 
        // else
        //if (calibrated<510 && calibrated>490){ calibrated=500; } //?  //Pare che una banda morta migliori le prestazioni...
    // last_values[i] = calibrated;
    // i = ++i%2;
    //if (calibrated<515 && calibrated>485){ calibrated=500; } //?  //Pare che una banda morta migliori le prestazioni...
}

/*
int Channel::calibrate_read() { 
    calibrated = (time-offset)*factor;
    if (calibrated>1000) {
        return 1000;
        } 
    if (calibrated<0) {
        return 0;
    }
    if (calibrated<515 && calibrated>485){ 
        return 500; 
    }
    return calibrated;
}*/

int Channel::read() { 
    return calibrated;
}


// ----------- PRINT ---------------------
void Channel::get_parameters() { printf("Offset:%8d  new_max:%6d  factor:%10f \n", offset,new_max,factor);}
void Channel::print() { printf("CH%d: %5d    ",number, calibrated); }
