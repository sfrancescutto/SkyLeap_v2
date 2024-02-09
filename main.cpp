/* 
Started by: Riccardo Medvescek
date: 28/2/23
Edited by: Stefano Francescutto
in December 2023

License: freeware
This project is intended as an assessment exercise for an examination. The code is complete and ready 
to be used, however, tuning of the PID coefficients and complementary filters has not been done. 
Mathematical simulation or empirical calibration work would be required.

Hardware setup 

MPU9150-----FRDM KL25Z:       ( /!\ beware: I/O pin with I2C capabilities needed)
 VDD     ->  3.3 V 
 SDA     ->  PTE0
 SCL     ->  PTE1
 GND     ->  GND
 others  ->  N.C.
RF reciever-FRDM KL25Z:       ( /!\ beware: I/O pin with interrupt capabilities needed)
 CH 1    ->  PTD3
 CH 2    ->  PTD2 
 CH 3    ->  PTD0 
 CH 4    ->  PTD5 
ESC PWM output:               ( /!\ beware: I/O pin with PWM capabilities needed)
 ESC1    ->  PTB0
 ESC2    ->  PTB1
 ESC3    ->  PTB2
 ESC4    ->  PTB3
Switches:
 SW1     ->  PTD4
 SW2     ->  PTA12
 SW3     ->  PTA4
 SW4     ->  PTA5
 SW5     ->  PTC8
 SW6     ->  PTC9
Shield's LEDs
 Green   ->  PTE20
 Blue    ->  PTE21
 Red     ->  PTE22
Others  
 Buzzer  ->  PTE23      ( /!\ beware: I/O pin with PWM capabilities needed)
 Button 1->             ( /!\ beware: I/O pin with interrupt capabilities needed)
 Button 2->             ( /!\ beware: I/O pin with interrupt capabilities needed)

SWITCHES assignments:
 SW 1     Serial output on off
 SW 2     Accelerometer-Gyroscope calibration
 SW 3     Magnetometer hand calibration
 SW 4     radio calibration
 SW 5     shutdown for activate main loop
 SW 6     Sound On-Off

||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
\\                                                                                  \\
\\   0   1        x                   nose down -> positive pitch                   \\
\\    \ /         ^                   right wing down -> negative roll              \\
\\     X          |                                                                 \\
\\    / \         |                                                                 \\
\\   2   3      z +-----> y                                                         \\
\\                                                                                  \\
\\  0 and 3 CW                                                                      \\
\\  1 and 2 CCW                                                                     \\
\\                                                                                  \\
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

last: the frequency of the main loop is set to 50 Hz. However, the program takes about 12 ms 
to make one cycle so it's already possible to raise the frequency to 80Hz. Of these 12 ms, about 
8 ms are needed to read data from the magnetometer in FUSE mode. Maybe it's possible to read 
directly the magnetometer data using the auxiliary SDA SCL pin on MPU9150 reducing the needed 
time drastically. In this case, the cycle time could be reduced to about 5ms. This could be useful 
to raise the frequency up to 200Hz or to add higher-level functionality.

Beware: simply raising the frequency value over 80 Hz would be useless in terms of cycle speed 
and could lead to bad error estimation due to wrong integration and derivation factor
*/


#include "mbed.h"
#include "Utils.h"

#define DEBUG 1

#ifdef DEBUG
#define SELECT_MODE() select_mode()
#else
#define SELECT_MODE()
#endif

//Serial port
// bool serialCom;
static BufferedSerial serial_port(USBTX, USBRX, 115200);

FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

int main() 
{   
    initialize();
    SELECT_MODE();
    main_loop();
}