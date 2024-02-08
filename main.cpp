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
#include "math.h"
#include <cstdint>
#include <cstdio>
#include "MPU9150.h"
//#include "Channel.h"  //commented because already included in calibration
#include "Calibration.h"
#include "Lights.h"
#include "FIFO_register.h"
#include "Quaternion.hpp"
#include "ReadSensors.h"

//TO ENABLE DEBUG
#include "debug.h"

#define PI          3.14159265
// #define FILTERING_ORDER 2
// #define FREQ        50.0*FILTERING_ORDER  //Hz    Change this value to change the frequency of the cycle  //NOTA VA VERIFICATO SE IL TELECONMANDO FUNZIONA ANCORA BENE
#define PERIOD      20000 //microseconds
// #define PERIOD2     PERIOD/FILTERING_ORDER
#define DEG2RAD     0.0174533 
#define RAD2DEG     57.2958
// #define GSCF        65.5        //Gyroscope Scaling Factor
// #define GSCF        131        //Gyroscope Scaling Factor
#define INT_SCAL    2607.59     //Scale radiant to int: 0.0174533 = 1Â° = 45   ||  PI = 180Â° = 8192

//default value
#define YAW         0
#define PITCH       1
#define ROLL        2
#define THROTTLE    3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define PERIODS_IN_10_SECONDS 10000000/PERIOD
#define PERIODS_IN_4_SECONDS  4000000/PERIOD

///USAGE: Same as printf -> DEBUGPRINT("%d, %d\n",N,M);
#ifdef DEBUG
#define DEBUG_PRINT(format, args...) printf (format, args)
#else
#define DEBUG_PRINT(format, args...) 
#endif

//calibration samples
int sample=200;

//raw data
// int16_t raw_acc[3];
// int16_t raw_gyr[3];
// int16_t raw_mag[3];

//elaborated data
// float   acc_angle[3];       //angle estimation from acc
// float   gyr_angle[3];       //angle estimation from acc
// double gyr_delta[3];
// float   alpha = 0.02;        //first complementary filter parameter
// float   beta  = 0.5;        //second complementary filter parameter
// float   gamma = 0;          //yaw speed complementary filter parameter //NOTA: ERA 0.7
// float   mag_str[3];         //intermediate variable of magnetic field
// float   mag[3];             //magnetometer componets on earth system
// float   pitch,roll,yaw;     //final extimeted pitch roll and yaw
// int     counter = 0;        //counter
double totalpitchf = 0.;
double totalrollf = 0.;
double totalyawf = 0.;

// int angle[3];
// int previous_yaw;
// int delta_yaw;
float   ang_speed[3];       //velocitÃ  angolari

//MEDIA MOBILE
// FifoReg fiforeg(8);
// int FifoRegNew[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// short int fifoRegIndex = 0;
// double average_delta_yaw;

//PID coefficients (Yaw, Pitch, Roll)
///NOTE: THEY NEED TO BE TUNED 
float Kp[3] = {4.0, 1.3, 1.3};   
float Ki[3] = {0.00, 0.00, 0.00}; 
float Kd[3] = {0, 18, 18};

//set point, errore, errore differenziale, errore integrale (Yaw, Pitch, Roll)
int set_point[3]  = {0, 0, 0};
int new_e[3]      = {0, 0, 0};
int prev_e[3]     = {0, 0, 0};
int delta_e[3]    = {0, 0, 0};
long sum_e[3]     = {0, 0, 0};
int correction[3] = {0, 0, 0};

Quaternion frontPosition(0.,1.,0.,0.);
Quaternion sidePosition(0.,0.,1.,0.);
Quaternion rotation;

//ESC declaration and variables
int power[4] = {1191, 1056, 1052, 1167};
PwmOut ESC1(PTB0);
PwmOut ESC2(PTB1);
PwmOut ESC3(PTB2);
PwmOut ESC4(PTB3);

//RF declaration and variables
Channel channel1(PTD3,1);
Channel channel2(PTD2,2);
Channel channel3(PTD0,3);
Channel channel4(PTD5,4);
int     default_offset[4] = {118, 53, 228, -12};
float   default_factor[4] = {1.244344, 1.375000, 1.726845, 1.196953};

//Switch declaration
DigitalIn SW1(PTD4,  PullUp);
DigitalIn SW2(PTA12, PullUp);
DigitalIn SW3(PTA4,  PullUp);
DigitalIn SW4(PTA5,  PullUp);
DigitalIn SW5(PTC8,  PullUp);

//Cycle Timer
Timer CycleTimer;
long CycleBegin, CycleEnd;
int CycleCounter=0;
int CycleTime;

//Serial port
bool serialCom;
static BufferedSerial serial_port(USBTX, USBRX, 115200);

FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

float test_mag[3] = {0, 0, 0};
int offset_acc_main[3] = {0, 0, 0};
int offset_gyr_main[3] = {0, 0, 0};
int hard_mag_main[3] = {0, 0, 0};
float soft_mag_main[3] = {0, 0, 0};

//_______________________________________________________________________________________________________________________________________
//
//|||||||||||||||||||||||||||||||||||||||||||||||||   MAIN   |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//_______________________________________________________________________________________________________________________________________
void provaRadiocomando(){
    int read_throttle = 0;
    //10 seconds of test
    for(int i = 0;i<PERIODS_IN_10_SECONDS;i++) {
        CycleTimer.start();

        channel1.calibrate();
        channel2.calibrate();
        channel3.calibrate();
        channel4.calibrate();

        //set point: pid objective ( * 1.82 = max 10Â°)
        set_point[0]  = channel1.read() - 500;
        set_point[1]  = channel2.read() - 500;
        set_point[2]  = channel4.read() - 500;
        read_throttle = channel3.read();

        DEBUG_PRINT("%d, %d, %d, %d\n",set_point[0], set_point[1], read_throttle, set_point[2]);

        CycleEnd = CycleTimer.elapsed_time().count();
        if (CycleTime < PERIOD) { //Fixed at 50Hz
            wait_us(int(PERIOD - CycleTime));
        } else 
            wait_us(PERIOD-CycleTime);
        CycleTimer.reset();
    }
    return;
}

void provaMotori(){
    while(true) {
        CycleTimer.start(); 

        int motore = 0;
        while(true) {
            motore = 0;
            power[0] = 1000;
            power[1] = 1000;
            power[2] = 1000;
            power[3] = 1000;

            do {
                printf("Motore? (0-3) (9 -> Torna al menu precedente)");
                scanf("%d",&motore);
                if (motore == 9) {
                    return;
                }
                if (motore < 0 || motore > 3) {
                    printf("\n%d: valore non valido\n", motore);
                }
                } while (motore < 0 || motore > 3);
                do {
                printf("Potenza da dare al motore %d? (1000-2000) ",motore);
                scanf("%d",&power[motore]);;
                if (power[motore] < 1000 || motore > 2000) {
                    printf("\n%d: valore non valido\n", power[motore]);
                }
                } while (power[motore] < 1000 || power[motore] > 2000);
            printf("Invio al motore %d la potenza di %d/2000...\n",motore ,power[motore]);

            //4 seconds of test
            for(int i = 0;i<PERIODS_IN_4_SECONDS;i++) {
                CycleTimer.start();
                ///TODO: PROBABLY REMOVE THIS v
                CycleBegin = CycleTimer.elapsed_time().count();

                ESC1.pulsewidth_us(power[0]);
                ESC2.pulsewidth_us(power[1]);
                ESC3.pulsewidth_us(power[2]);
                ESC4.pulsewidth_us(power[3]);

                CycleTime = CycleTimer.elapsed_time().count();

                if (CycleTime < PERIOD) {
                    wait_us(int(PERIOD - CycleTime));
                } else 
                    wait_us(PERIOD-CycleTime);
                CycleTimer.reset();
            }
        }
    }
}

void provaSensori(){
    for(int k = 0;k>-1;k++) {
    readSensors(offset_acc_main, offset_gyr_main, CycleTimer, totalrollf, totalpitchf, totalyawf);

    //reset
    totalpitchf = 0.;
    totalrollf  = 0.;

    auto CycleEnd = CycleTimer.elapsed_time().count();
    if (CycleEnd < PERIOD) {
        wait_us(int(PERIOD - CycleEnd));
    } else 
        printf("!!!WARNING!!! Selected frequency is too high for the device to work properly!\n");
    CycleTimer.reset();
    }
    return;
    // float tot_gyr_angle[3] = {0., 0., 0.};
    // float purely_gyroscopic_angle_est[2] = {0., 0.};
    // double totalpitchf = 0.;
    // double totalrollf = 0.;
    // double totalyawf = 0.;

    // short counter = 0;

    // double gyroscope_conversion_constant = 1/(FREQ*GSCF);

    // for(int k = 0;k>-1;k++) {
    //     CycleTimer.start();

    //     readAccelData(raw_acc);
    //     readGyroData(raw_gyr);

    //     // Accelerometer offset correction
    //     // If the starting position of the MPU-9150 is horizontal,
    //     // the expected values for X and Y acceleration are zero, 
    //     // while the expected value for Z acceleration is g.
    //     raw_acc[0] -= offset_acc_main[0];
    //     raw_acc[1] -= offset_acc_main[1];
    //     raw_acc[2] -= offset_acc_main[2];

    //     // Gyroscope offset correction
    //     raw_gyr[0] -= offset_gyr_main[0];
    //     raw_gyr[1] -= offset_gyr_main[1];
    //     raw_gyr[2] -= offset_gyr_main[2];

    //     // get angle aproximation by accelerometer vector
    //     acc_angle[PITCH]    = -atan2(  raw_acc[X],  sqrtf( raw_acc[Y] * raw_acc[Y]  +  raw_acc[Z] * raw_acc[Z] )  ) ;
    //     acc_angle[ROLL]     = -atan2(  raw_acc[Y],  sqrtf( raw_acc[X] * raw_acc[X]  +  raw_acc[Z] * raw_acc[Z] )  ) ;

    //     // get angle aproximation by angular speed integration (degrees to radiant (0.0174533)) (GSCF (65.5))
    //     gyr_angle[Y]   += ( raw_gyr[Y] * gyroscope_conversion_constant) * DEG2RAD;
    //     gyr_angle[X]   += (-raw_gyr[X] * gyroscope_conversion_constant) * DEG2RAD;
    //     gyr_angle[Z]   += ( raw_gyr[Z] * gyroscope_conversion_constant); //DEGREES

    //     //To show estimation based on gyroscope only
    //     purely_gyroscopic_angle_est[X]   -= raw_gyr[X] * gyroscope_conversion_constant; //DEGREES
    //     purely_gyroscopic_angle_est[Y]   += raw_gyr[Y] * gyroscope_conversion_constant; //DEGREES

    //     //For YAW estimation
    //     tot_gyr_angle[Y]   += ( raw_gyr[Y] * gyroscope_conversion_constant); //DEGREES
    //     tot_gyr_angle[Z]   += ( raw_gyr[Z] * gyroscope_conversion_constant); //DEGREES

    //     // compensate gyro angle with accelerometer angle in a complementary filter (accelerometer -> LF ; gyroscope -> HF)
    //     gyr_angle[Y]    = gyr_angle[Y] * alpha + acc_angle[PITCH] * (1-alpha);
    //     gyr_angle[X]    = gyr_angle[X] * alpha + acc_angle[ROLL] * (1-alpha);

    //     // compensate yawing motion in angle estimation
    //     gyr_angle[Y]   += gyr_angle[X] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD );
    //     gyr_angle[X]   -= gyr_angle[Y] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD );

    //     purely_gyroscopic_angle_est[X]   -= purely_gyroscopic_angle_est[Y] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD ); //DEGREES
    //     purely_gyroscopic_angle_est[Y]   += purely_gyroscopic_angle_est[X] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD ); //DEGREES

    //     // get pitch and roll (low pass complementary filter)
    //     pitch  = pitch * beta + gyr_angle[Y] * (1-beta);
    //     roll   = roll  * beta + gyr_angle[X] * (1-beta);
                
    //     totalpitchf += pitch;
    //     totalrollf  += roll;

    //     if (++counter < FILTERING_ORDER) {
    //         CycleEnd = CycleTimer.elapsed_time().count();
    //         if (CycleEnd < PERIOD2*counter) {
    //             wait_us(int(PERIOD2*counter - CycleEnd));
    //         } else 
    //             printf(" !!!WARNING!!! Filtering Order is too high for the device to work properly!\n");
    //         continue;
    //     }

    //     ///TODO: TEST THIS! 
    //     totalpitchf /= FILTERING_ORDER;
    //     totalrollf  /= FILTERING_ORDER;

    //     totalyawf += tot_gyr_angle[Z]*cos(totalrollf/FILTERING_ORDER) - tot_gyr_angle[Y]*sin(totalrollf/FILTERING_ORDER);
    //     if (totalyawf > 360) {
    //         totalyawf -= 360;
    //     } else if (totalyawf < -360) {
    //         totalyawf += 360;
    //     }

    //     tot_gyr_angle[Y] = 0.;
    //     tot_gyr_angle[Z] = 0.;

    //     DEBUG_PRINT("FILT:\nP: %d\nR: %d\nY: %d\n\n",int(totalpitchf*RAD2DEG), int(totalrollf*RAD2DEG), int(totalyawf));
    //     DEBUG_PRINT("GYR:\nP: %d\nR: %d\n\n",int(purely_gyroscopic_angle_est[Y]),int(purely_gyroscopic_angle_est[X]));
    //     DEBUG_PRINT("ACC:\nP: %d\nR: %d\n\n",int(acc_angle[PITCH]*RAD2DEG),int(acc_angle[ROLL]*RAD2DEG));

    //     totalpitchf = 0.;
    //     totalrollf  = 0.;

    //     counter = 0;

    //     CycleEnd = CycleTimer.elapsed_time().count();
    //     if (CycleEnd < PERIOD) {
    //         wait_us(int(PERIOD - CycleEnd));
    //     } else 
    //         printf("!!!WARNING!!! Selected frequency is too high for the device to work properly!\n");
    //     CycleTimer.reset();
    // }
    // return;
}

int main() 
{   
    signal_start();//SOUND AND LIGHT EFFECT
    ThisThread::sleep_for(500ms);
    //SERIAL DECLARATION 
    if (not SW1){
        printf("\nSWITCH_1 ON:\nSerial comunication ON\n\nSTARTING SETUP \n\n");
        serialCom = true;
        signal_ok();
		ThisThread::sleep_for(1s);//SOUND AND LIGHT EFFECT
    } else { serialCom = false; }


    //INIZIALIZZAZIONE 
    initMPU9150();
    initAK8975A(test_mag);
    if (serialCom) { printf("MPU-9150 and AK8975A initialized \n\n");}
    signal_ok();ThisThread::sleep_for(100ms);//SOUND AND LIGHT EFFECT

    ESC1.period_ms(20);
    ESC2.period_ms(20);
    ESC3.period_ms(20);
    ESC4.period_ms(20);
    ESC1.pulsewidth_us(1000);
    ESC2.pulsewidth_us(1000);
    ESC3.pulsewidth_us(1000);
    ESC4.pulsewidth_us(1000);

    //CALIBRAZIONE ACC E GYRO
    if (not SW2){
        if (serialCom) { printf("SWITCH_2 ON\n\nSTARTING ACC/GYRO CALIBRATION IN 3 SECONDS \nPLEASE DON'T MOVE! \n\n"); }
        signal_calibration(1);//SOUND AND LIGHT EFFECT
        bool movement = true;
        while (movement) {
            movement = acc_gyr_calibration(sample, &offset_acc_main[0], &offset_gyr_main[0]);
            if (movement) {
                signal_error(); //SOUND AND LIGHT EFFECT
                if (serialCom){printf("MOVEMENT DETECTED: Restarting \n\n");}
                signal_calibration(1);//SOUND AND LIGHT EFFECT
            }
        }
        signal_stop_calibration(1); ThisThread::sleep_for(1s);//SOUND AND LIGHT EFFECT
        if (serialCom) {printf("OFFSET CALCULATED \nACC: %d %d %d   GYRO: %d %d %d \n\n\n", offset_acc_main[0],offset_acc_main[1],offset_acc_main[2],offset_gyr_main[0],offset_gyr_main[1],offset_gyr_main[2]); }
    } else {
        if (serialCom) {printf("SWITCH_2 OFF\n\n");}
        default_mag();
    }

    //CALIBRAZIONE MAG
    if (not SW3){
        signal_calibration(2);//SOUND AND LIGHT EFFECT
        if (serialCom) {printf("SWITCH 3 ON\n\nMAGNETOMETER CALIBRATION: Rotate DRONE in all directions \n\n\n"); }

        mag_calibration(hard_mag_main, soft_mag_main);
        if (serialCom) {printf("Hard Iron OFFSET CALCULATED\nHARD IRON: %5d %5d %5d\nsoft IRON: %8f %8f %8f\n\n\n",
                                hard_mag_main[0],hard_mag_main[1],hard_mag_main[2],soft_mag_main[0],soft_mag_main[1],soft_mag_main[2]);}
        signal_stop_calibration(2);//SOUND AND LIGHT EFFECT
    } else {
        if (serialCom) {printf("SWITCH 3 OFF\n\n");}
    }
    
    //CALIBRAZIONE RADIO
    if (not SW4){
        signal_calibration(3);//SOUND AND LIGHT EFFECT
        if (serialCom) {printf("SWITCH 4 ON\n\nRADIO INPUT CALIBRATION: Rotate JOYSTICKS in all direction \n\n\n"); }

        for( int j=0; j<300; j++) {
            ThisThread::sleep_for(20ms);
            channel1.find_interval();
            channel2.find_interval();
            channel3.find_interval();
            channel4.find_interval();
        }
        channel1.calculate_calibration();
        ThisThread::sleep_for(20ms);
        channel2.calculate_calibration();
        ThisThread::sleep_for(20ms);
        channel3.calculate_calibration();
        ThisThread::sleep_for(20ms);
        channel4.calculate_calibration();
        ThisThread::sleep_for(20ms);

        if (serialCom) {printf("Radio Input calibrated\n");
            ThisThread::sleep_for(100ms);
            channel1.get_parameters();
            ThisThread::sleep_for(100ms);
            channel2.get_parameters();
            ThisThread::sleep_for(100ms); 
            channel3.get_parameters();
            ThisThread::sleep_for(100ms);
            channel4.get_parameters();
            ThisThread::sleep_for(100ms);
        }
        signal_stop_calibration(3);//SOUND AND LIGHT EFFECT
    } else {
        if (serialCom) {printf("SWITCH 4 OFF\n");}
        channel1.channel_default_value(default_offset[0], default_factor[0]);
        channel2.channel_default_value(default_offset[1], default_factor[1]);
        channel3.channel_default_value(default_offset[2], default_factor[2]);
        channel4.channel_default_value(default_offset[3], default_factor[3]);
    }
 
    //signal_ready();ThisThread::sleep_for(1s);//SOUND AND LIGHT EFFECT
    //shutup();ThisThread::sleep_for(1s);//SHUTUP: if not called the buzzer will keep producing noise

    //MOTOR 
    if (serialCom) {printf("STARTING ESC \n\n");}
    //Nota: Diversa da quella del loop di controllo

    if (serialCom) {printf("SHUT SW5 TO START \n\n");}
    while(not SW5) {
        ThisThread::sleep_for(100ms);
    }
    ThisThread::sleep_for(200ms);

    ESC1.pulsewidth_us(power[0]);
    ESC2.pulsewidth_us(power[1]);
    ESC3.pulsewidth_us(power[2]);
    ESC4.pulsewidth_us(power[3]);
    ThisThread::sleep_for(2s);

    power[0] = 1000;
    power[1] = 1000;
    power[2] = 1000;
    power[3] = 1000;
    if (serialCom) {printf("START \n\n");}
    //Remove comment to allow runtime selection of proportional contant for PID filter.
    /*
    printf("Inserisci i valori di Kp:\n");
    scanf("%f",&Kp[0]);
    scanf("%f",&Kp[1]);
    scanf("%f",&Kp[2]);
    printf("Valori inseriti:\nY: %f, R: %f, P: %f\n", Kp[YAW],Kp[ROLL],Kp[PITCH]);
    */
    ThisThread::sleep_for(100ms);
    
    int mode = 0;
    do {
    printf("1 --> Prova Radiocomando\n2 --> Prova Motori\n3 --> Prova Sensori\n9 --> Avvio normale\n");
    scanf("%d",&mode);
    switch(mode) {
        case 9: 
            printf("Avvio normale...\n");
            break;
        case 1: 
            printf("Prova Radiocomando...\n");
            provaRadiocomando();
            break;
        case 2: 
            printf("Prova Motori...\n");
            provaMotori();
            break;
        case 3: 
            printf("Prova Sensori...\n");
            provaSensori();
            break;
        default:
            printf("Invalid mode, please try again.\n");
            break;
    }
    } while (mode != 9);

//_______________________________________________________________________________________________________________________________________
//
//|||||||||||||||||||||||||||||||||||||||||||||||||  LOOP  |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//_______________________________________________________________________________________________________________________________________
    
    // int totalpitch = 0;
    // int totalroll = 0;

    // float tot_gyr_angle[3] = {0., 0., 0.};

    // short counter = 0;

    // double gyroscope_conversion_constant = 1/(FREQ*GSCF);
    int read_throttle = 0;
/* */

    CycleTimer.start();
    while(true) {   
        readSensors(offset_acc_main, offset_gyr_main, CycleTimer, totalrollf, totalpitchf, totalyawf);
        // read raw data from sensors
        // readAccelData(raw_acc);
        // readGyroData(raw_gyr);

        // // if(counter == 0)
        // //     readMagData_calibr(raw_mag, hard_mag_main, soft_mag_main); //same of readMagData(raw_mag) with in-flight autocalibration added

        // // Accelerometer offset correction
        // raw_acc[0] -= offset_acc_main[0];
        // raw_acc[1] -= offset_acc_main[1];
        // raw_acc[2] -= offset_acc_main[2];

        // // Gyroscope offset correction
        // raw_gyr[0] -= offset_gyr_main[0];
        // raw_gyr[1] -= offset_gyr_main[1];
        // raw_gyr[2] -= offset_gyr_main[2];

        // // Magnetometer hard and soft iron correction
        // // Per il momento ignoriamo la bussola
        // // raw_mag[0] = (raw_mag[0] - hard_mag_main[0]) / soft_mag_main[0];      // per correggere l'errore si è sviluppato il modello
        // // raw_mag[1] = (raw_mag[1] - hard_mag_main[1]) / soft_mag_main[1];      // X_rilevato = X_offset + alpha * x_corretto
        // // raw_mag[2] = (raw_mag[2] - hard_mag_main[2]) / soft_mag_main[1];      // per risolvere soft iron si normalizza a 100
    
        // // correct axsis orientation of magnetometer (magnetometer and accelerometer don't share same axis)
        // // mag_str[0] =  raw_mag[1];
        // // mag_str[1] = -raw_mag[0];
        // // mag_str[2] =  raw_mag[2];

        // // get angle aproximation by accelerometer vector
        // acc_angle[Y]    = -atan2(  raw_acc[X],  sqrtf( raw_acc[Y] * raw_acc[Y]  +  raw_acc[Z] * raw_acc[Z] )  ) ;
        // acc_angle[X]    = -atan2(  raw_acc[Y],  sqrtf( raw_acc[X] * raw_acc[X]  +  raw_acc[Z] * raw_acc[Z] )  ) ;

        // // get angle aproximation by angular speed integration (degrees to radiant (0.0174533)) (GSCF (65.5))
        // gyr_angle[Y]   += ( raw_gyr[Y] * gyroscope_conversion_constant) * DEG2RAD;
        // gyr_angle[X]   += (-raw_gyr[X] * gyroscope_conversion_constant) * DEG2RAD;
        // gyr_angle[Z]   += ( raw_gyr[Z] * gyroscope_conversion_constant); //DEGREES

        // //For YAW estimation
        // tot_gyr_angle[Y]   += ( raw_gyr[Y] * gyroscope_conversion_constant); //DEGREES
        // tot_gyr_angle[Z]   += ( raw_gyr[Z] * gyroscope_conversion_constant); //DEGREES

        // // compensate gyro angle with accelerometer angle in a complementary filter (accelerometer -> LF ; gyroscope -> HF)
        // gyr_angle[Y]    = gyr_angle[Y] * alpha + acc_angle[PITCH] * (1-alpha);
        // gyr_angle[X]    = gyr_angle[X] * alpha + acc_angle[ROLL] * (1-alpha);

        // // compensate yawing motion in angle estimation
        // gyr_angle[Y]   += gyr_angle[X] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD ); //DEGREES
        // gyr_angle[X]   -= gyr_angle[Y] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD ); //DEGREES

        // // get pitch and roll (low pass complementary filter)
        // pitch  = pitch * beta + gyr_angle[Y] * (1-beta);
        // roll   = roll  * beta + gyr_angle[X] * (1-beta);

        // totalpitchf += pitch;
        // totalrollf  += roll;
    
        // // get yaw estimation by magnetometer 
        // // mag[0] = mag_str[0] * cos(pitch) + mag_str[1] * sin(roll) * sin(pitch) - mag_str[2] * cos(roll) * sin(pitch);
        // // mag[1] = mag_str[1] * cos(roll)  + mag_str[2] * sin(roll);

        // // yaw    = atan2(mag[1], mag[0]) ;
        
        // // scaling float values to int -> PI = 180Â° = 8192
        // // good sensitivity and range 
        // // angle[YAW]   = yaw   *INT_SCAL;//( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL;
        // //angle[PITCH] = pitch *INT_SCAL;
        // //angle[ROLL]  = roll  *INT_SCAL;
        // totalpitch += pitch *INT_SCAL;
        // totalroll  += roll  *INT_SCAL;

        // //printf("\nRollio: %d\nBeccheggio: %d\nImbardata: %d",angle[ROLL], angle[PITCH], angle[YAW]);

        
        // // hard to use directly the yaw angle to correct the drift
        // // alternative: get the yaw anglular speed from a filtered compass which is driftless but noisy
        // // and implementing another complementary filter: LP from compass and HP from gyroscope
        // // In this way we obtain the asymptotic behavior to the compass angle

        // // TO BE CORRECTED
        // // previous angle should be filtered like current angle
        // // if (angle[YAW] > 8192 && previous_yaw < -8192) {
        // //     delta_yaw = (angle[YAW] - previous_yaw - 8192);
        // // } else if(angle[YAW]< -8192 && previous_yaw> 8192) {
        // //     delta_yaw = (angle[YAW] - previous_yaw + 8192);
        // // } else {
        // //     delta_yaw = (previous_yaw - angle[YAW]);
        // // }
        // // previous_yaw = angle[YAW];

        // /*
        // Nota: secondo me, per calcolare adeguatamente l'imbardata media è il caso di fare così:
        // Se i campioni della media sono tutti superiori a 360°, li sovrascriviamo come se fossero nei pressi dello zero
        // Stesso discorso per angoli negativi molto grandi. 
        // Che va anche bene forse, ma come la mettiamo per il giroscopio?
        // */

        // //NOTA: Siccome ho deciso di implementare un altro modo, questo lo commento
        // //average_yaw = fiforeg.FifoReg_shift_and_m_av(delta_yaw);
        
        // //NOTA: Questo calcola la media sugli ultimi 8 valori:
        // // FifoRegNew[fifoRegIndex] = delta_yaw;
        // // average_delta_yaw = 0.;
        // // for (int i = 0;i < 8; i++)
        // //     average_delta_yaw += FifoRegNew[i];
        // // average_delta_yaw /= 8;
        // // fifoRegIndex++;
        // // fifoRegIndex = fifoRegIndex % 8;

        // //complementary filter
        // //NOTA: Questo pare dare una posizione angolare, più che una velocità
        // // ang_speed[YAW] = (gamma) * average_delta_yaw + (1 - gamma) * ( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL;
        
        // if (++counter < FILTERING_ORDER) {
        //     CycleEnd = CycleTimer.elapsed_time().count();
        //     if (CycleEnd < PERIOD2*counter) {
        //         wait_us(int(PERIOD2*counter - CycleEnd));
        //     } else 
        //         printf(" !!!WARNING!!! Filtering Order is too high for the device to work properly!\n");
        //     continue;
        // }

        // use the calibration factor on RF recieved data
        channel1.calibrate();
        channel2.calibrate();
        channel3.calibrate();
        channel4.calibrate();

        //set point: pid objective ( * 1.82 = max 10Â°)
        set_point[0]  = (channel1.read() - 500) * 1.82;
        set_point[1]  = (channel2.read() - 500) * 1.82;
        set_point[2]  = (channel4.read() - 500) * 1.82;

        // NEEDS TO BE TESTED!
        // totalpitchf /= FILTERING_ORDER;
        // totalrollf /= FILTERING_ORDER;
        // IF ^ WORKS, down here we can remove the /FILTERING_ORDER
        // totalyawf += tot_gyr_angle[Z]*cos(totalrollf/FILTERING_ORDER) - tot_gyr_angle[Y]*sin(totalrollf/FILTERING_ORDER);
        
        // // Please note that the following can also be done with % operator
        // // but also note that that would require totalyawf to be integer. 
        // if (totalyawf > 360) {
        //     totalyawf -= 360;
        // } else if (totalyawf < -360) {
        //     totalyawf += 360;
        // }

        // tot_gyr_angle[Y] = 0.;
        // tot_gyr_angle[Z] = 0.;

        // totalpitch /= FILTERING_ORDER;
        // totalroll /= FILTERING_ORDER;

        ///IMPORTANT:
        //Please, note that the following has to be carefully 
        //corrected, to allow the drone to follow the yaw set point as desired

        //new error
        new_e[YAW]   = ang_speed[YAW]   - set_point[YAW];
        //new_e[PITCH] = angle[PITCH]     - set_point[PITCH];
        //new_e[ROLL]  = angle[ROLL]      - set_point[ROLL];
        new_e[PITCH] = totalpitchf     - set_point[PITCH];
        new_e[ROLL]  = totalrollf      - set_point[ROLL];

        //integrative factor
        sum_e[YAW]   += new_e[YAW];
        sum_e[PITCH] += new_e[PITCH];
        sum_e[ROLL]  += new_e[ROLL];

        // set maximum and minimum
        sum_e[YAW]   = cutOff(sum_e[YAW],   -40/Ki[YAW],   40/Ki[YAW]);
        sum_e[PITCH] = cutOff(sum_e[PITCH], -40/Ki[PITCH], 40/Ki[PITCH]);
        sum_e[ROLL]  = cutOff(sum_e[ROLL],  -40/Ki[ROLL],  40/Ki[ROLL]);

        //derivative factor
        delta_e[YAW]   = new_e[YAW]   - prev_e[YAW];
        delta_e[PITCH] = new_e[PITCH] - prev_e[PITCH];
        delta_e[ROLL]  = new_e[ROLL]  - prev_e[ROLL];

        //update previous error
        prev_e[YAW]   = new_e[YAW];
        prev_e[PITCH] = new_e[PITCH];
        prev_e[ROLL]  = new_e[ROLL];

        //calculate correction 
        correction[YAW]   = (cutOffInt((new_e[YAW]   * Kp[YAW])   + (sum_e[YAW]   * Ki[YAW])   + (delta_e[YAW]   * Kd[YAW]), -500, +500))*0.12 ;
        correction[PITCH] = (cutOffInt((new_e[PITCH] * Kp[PITCH]) + (sum_e[PITCH] * Ki[PITCH]) + (delta_e[PITCH] * Kd[PITCH]), -500, +500))*0.12 ;
        correction[ROLL]  = (cutOffInt((new_e[ROLL]  * Kp[ROLL])  + (sum_e[ROLL]  * Ki[ROLL])  + (delta_e[ROLL]  * Kd[ROLL]), -500, +500))*0.12 ;

        //THROTTLE POWER CALCULATION
        read_throttle = channel3.read();

       //NOTA che secondo me va ancora appurato che questo sia un modo furbo di proseguire. Io penso che preferirei ragionare con qualcosa di più strettamente legato al motore
       if (read_throttle>50){
            power[0] = 1190 + read_throttle + correction[ROLL] + correction[PITCH] + correction[YAW];
            power[1] = 1055 + read_throttle - correction[ROLL] + correction[PITCH] - correction[YAW];
            power[2] = 1050 + read_throttle + correction[ROLL] - correction[PITCH] - correction[YAW];
            power[3] = 1168 + read_throttle - correction[ROLL] - correction[PITCH] + correction[YAW];
        } else {
            power[0] = 1000;
            power[1] = 1000;
            power[2] = 1000;
            power[3] = 1000;
        }

        //ESC OUTPUT        
        ESC1.pulsewidth_us(power[0]);
        ESC2.pulsewidth_us(power[1]);
        ESC3.pulsewidth_us(power[2]);
        ESC4.pulsewidth_us(power[3]);
        
        // Green Led Running program (toggle every 100 cycles)
        // CycleCounter++;
        // if (CycleCounter==100) { CycleCounter=0; on_off(2); }

        // Every Cycle should be 20ms long: match 50Hz frequency by waiting excess time
        CycleEnd = duration_cast<std::chrono::microseconds>(CycleTimer.elapsed_time()).count();
        if (CycleEnd < PERIOD) {
            wait_us(int(PERIOD - CycleTime));
        } else {
            printf("!!! WARNING !!!\nWORKING FREQUENCY IS TOO HIGH, SYSTEM MAY NOT WORK CORRECTLY\n"); //DEBUG oppure no, non lo so
        }
        // totalpitch = 0;
        // totalroll = 0;
        
        totalpitchf = 0.;
        totalrollf  = 0.;

        // counter = 0;
        CycleTimer.reset(); //reset does NOT stop the timer, it just resets its counter to zero.
    }
}

/* print collection

        //channel1.print();channel2.print();channel3.print();channel4.print();

        //printf("%6d %6d %6d   \n",raw_mag[0],raw_mag[1],raw_mag[2]);
        
        //printf("%6d %6d %6d  -  %6d %6d %6d  -  %6d %6d %6d\n",raw_acc[0],raw_acc[1],raw_acc[2],raw_gyr[0],raw_gyr[1],raw_gyr[2],raw_mag[0],raw_mag[1],raw_mag[2]);
        
        //printf("%11f, %11f, %11f  \n",pitch, roll, yaw);

        //printf("%11f, %11f, %11f       %11f, \n",pitch* 180 /PI, roll* 180 /PI, yaw* 180 /PI, averageYaw);

        //printf("%13f, %13f        %9f, %9f, %9f\n",mag[0], mag[1], pitch, roll, yaw);
        
        //printf("%6d %6d %6d    -    %5f\n",raw_mag[0],raw_mag[1],raw_mag[2],intensity);
        
        //fiforeg.FifoReg_print();

        //printf("HARD IRON: %5d %5d %5d    soft IRON: %6f %6f %6f \n", hard_mag_main[0],hard_mag_main[1],hard_mag_main[2], soft_mag_main[0],soft_mag_main[1],soft_mag_main[2]);

*/