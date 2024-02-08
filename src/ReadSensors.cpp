#include "ReadSensors.h"
#include "mbed.h"
#include "MPU9150.h"

//TO ENABLE DEBUG:
#include "debug.h"

#define PI          3.14159265
#define FILTERING_ORDER 2
#define FREQ        50.0*FILTERING_ORDER  //Hz    Change this value to change the frequency of the cycle  //NOTA VA VERIFICATO SE IL TELECONMANDO FUNZIONA ANCORA BENE
#define PERIOD      20000 //microseconds
#define PERIOD2     PERIOD/FILTERING_ORDER
#define DEG2RAD     0.0174533 
#define RAD2DEG     57.2958
// #define GSCF        65.5        //Gyroscope Scaling Factor
#define GSCF        131        //Gyroscope Scaling Factor
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

//raw data
int16_t raw_acc[3];
int16_t raw_gyr[3];
int16_t raw_mag[3];

//elaborated data
float   acc_angle[3];       //angle estimation from acc
float   gyr_angle[3];       //angle estimation from acc
double gyr_delta[3];
float   alpha = 0.02;        //first complementary filter parameter
float   beta  = 0.5;        //second complementary filter parameter
float   gamma = 0;          //yaw speed complementary filter parameter //NOTA: ERA 0.7
float   mag_str[3];         //intermediate variable of magnetic field
float   mag[3];             //magnetometer componets on earth system
float   pitch,roll,yaw;     //final extimeted pitch roll and yaw
int     counter = 0;        //counter

int angle[3];
int previous_yaw;
int delta_yaw;
float   ang_speed[3];       //velocitÃ  angolari

//MEDIA MOBILE
// FifoReg fiforeg(8);
// int FifoRegNew[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// short int fifoRegIndex = 0;
// double average_delta_yaw;

void readSensors(int offset_acc_main[], int offset_gyr_main[], Timer CycleTimer, double& totalrollf, double& totalpitchf, double& totalyawf) {
    float tot_gyr_angle[3] = {0., 0., 0.};
    float purely_gyroscopic_angle_est[2] = {0., 0.};
    // double totalpitchf = 0.;
    // double totalrollf = 0.;
    // double totalyawf = 0.;

    double gyroscope_conversion_constant = 1/(FREQ*GSCF);

    for(short counter = 0; counter <= FILTERING_ORDER; ++ counter) {
        CycleTimer.start();

        readAccelData(raw_acc);
        readGyroData(raw_gyr);

        // Accelerometer offset correction
        // If the starting position of the MPU-9150 is horizontal,
        // the expected values for X and Y acceleration are zero, 
        // while the expected value for Z acceleration is g.
        raw_acc[0] -= offset_acc_main[0];
        raw_acc[1] -= offset_acc_main[1];
        raw_acc[2] -= offset_acc_main[2];

        // Gyroscope offset correction
        raw_gyr[0] -= offset_gyr_main[0];
        raw_gyr[1] -= offset_gyr_main[1];
        raw_gyr[2] -= offset_gyr_main[2];

        // get angle aproximation by accelerometer vector
        acc_angle[PITCH]    = -atan2(  raw_acc[X],  sqrtf( raw_acc[Y] * raw_acc[Y]  +  raw_acc[Z] * raw_acc[Z] )  ) ;
        acc_angle[ROLL]     = -atan2(  raw_acc[Y],  sqrtf( raw_acc[X] * raw_acc[X]  +  raw_acc[Z] * raw_acc[Z] )  ) ;

        // get angle aproximation by angular speed integration (degrees to radiant (0.0174533)) (GSCF (65.5))
        gyr_angle[Y]   += ( raw_gyr[Y] * gyroscope_conversion_constant) * DEG2RAD;
        gyr_angle[X]   += (-raw_gyr[X] * gyroscope_conversion_constant) * DEG2RAD;
        gyr_angle[Z]   += ( raw_gyr[Z] * gyroscope_conversion_constant); //DEGREES

        //To show estimation based on gyroscope only
        purely_gyroscopic_angle_est[X]   -= raw_gyr[X] * gyroscope_conversion_constant; //DEGREES
        purely_gyroscopic_angle_est[Y]   += raw_gyr[Y] * gyroscope_conversion_constant; //DEGREES

        //For YAW estimation
        tot_gyr_angle[Y]   += ( raw_gyr[Y] * gyroscope_conversion_constant); //DEGREES
        tot_gyr_angle[Z]   += ( raw_gyr[Z] * gyroscope_conversion_constant); //DEGREES

        // compensate gyro angle with accelerometer angle in a complementary filter (accelerometer -> LF ; gyroscope -> HF)
        gyr_angle[Y]    = gyr_angle[Y] * alpha + acc_angle[PITCH] * (1-alpha);
        gyr_angle[X]    = gyr_angle[X] * alpha + acc_angle[ROLL] * (1-alpha);

        // compensate yawing motion in angle estimation
        gyr_angle[Y]   += gyr_angle[X] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD );
        gyr_angle[X]   -= gyr_angle[Y] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD );

        purely_gyroscopic_angle_est[X]   -= purely_gyroscopic_angle_est[Y] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD ); //DEGREES
        purely_gyroscopic_angle_est[Y]   += purely_gyroscopic_angle_est[X] * sin( raw_gyr[Z] * gyroscope_conversion_constant * DEG2RAD ); //DEGREES

        // get pitch and roll (low pass complementary filter)
        pitch  = pitch * beta + gyr_angle[Y] * (1-beta);
        roll   = roll  * beta + gyr_angle[X] * (1-beta);
                
        totalpitchf += pitch;
        totalrollf  += roll;

        auto CycleEnd = CycleTimer.elapsed_time().count();
        if (CycleEnd < PERIOD2*counter) {
            wait_us(int(PERIOD2*counter - CycleEnd));
        } else 
            printf(" !!!WARNING!!! Filtering Order is too high for the device to work properly!\n");
    }

    totalpitchf /= FILTERING_ORDER;
    totalrollf  /= FILTERING_ORDER;

    totalyawf += tot_gyr_angle[Z]*cos(totalrollf/FILTERING_ORDER) - tot_gyr_angle[Y]*sin(totalrollf/FILTERING_ORDER);
    if (totalyawf > 360) {
        totalyawf -= 360;
    } else if (totalyawf < -360) {
        totalyawf += 360;
    }

    tot_gyr_angle[Y] = 0.;
    tot_gyr_angle[Z] = 0.;

    DEBUG_PRINT("FILT:\nP: %d\nR: %d\nY: %d\n\n",int(totalpitchf*RAD2DEG), int(totalrollf*RAD2DEG), int(totalyawf));
    DEBUG_PRINT("GYR:\nP: %d\nR: %d\n\n",int(purely_gyroscopic_angle_est[Y]),int(purely_gyroscopic_angle_est[X]));
    DEBUG_PRINT("ACC:\nP: %d\nR: %d\n\n",int(acc_angle[PITCH]*RAD2DEG),int(acc_angle[ROLL]*RAD2DEG));

    // totalpitchf = 0.;
    // totalrollf  = 0.;

    // counter = 0;

    // auto CycleEnd = CycleTimer.elapsed_time().count();
    // if (CycleEnd < PERIOD) {
    //     wait_us(int(PERIOD - CycleEnd));
    // } else 
    //     printf("!!!WARNING!!! Selected frequency is too high for the device to work properly!\n");
    // CycleTimer.reset();
// }
    return;
}