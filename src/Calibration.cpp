#include "Calibration.h"
#include "mbed.h"
//#include "Channel.h"

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     //X axis
#define Y           1     //Y axis
#define Z           2     //Z axis

// ERRORI 
//-----------------------------
int offset_acc[3];      //accelerometer offset
int offset_gyr[3];      //gyroscope offset
int hard_mag[3];        //hard iron offset 
float soft_mag[3];        //soft iron deformation, normalized


//-----------------------------
//max and min magnetic rilevations
int     mag_max[3]      = {-32768,-32768,-32768};
int     mag_min[3]      = {32767,32767,32767};

int default_hard_mag[3] = {30, 16, -71};
float default_soft_mag[3] = {1.210000, 1.220000, 1.150000};

//Timer serviceT;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------Acc and Gyro sensors calibration---------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool acc_gyr_calibration(int sample, int* acc_offset, int* gyr_offset) {
    int bound = 400;                    //movement detection limit

    int16_t acc_data[3] = {0,0,0};      //input data from MPU
    int16_t gyr_data[3] = {0,0,0};      
    long    test_acc[3] = {0,0,0};      //accumulate variable
    long    test_gyr[3] = {0,0,0};
    int     max[3]      = {-32768,-32768,-32768};
    int     min[3]      = {32767,32767,32767};
    int     delta[3]    = {0,0,0};

    for (int i=0; i<sample; i++) {
        readAccelData(acc_data);        //read data from mpu
        readGyroData(gyr_data);
        
        test_acc[X] += acc_data[X];     //accumulate
        test_acc[Y] += acc_data[Y];
        test_acc[Z] += acc_data[Z];

        test_gyr[X] += gyr_data[X];
        test_gyr[Y] += gyr_data[Y];
        test_gyr[Z] += gyr_data[Z];

        if (acc_data[X] > max[X]) {     //check for max and min update
            max[X] = acc_data[X];
        } else if (acc_data[X] < min[X]) {
            min[X]=acc_data[X]; }
        if (acc_data[Y] > max[Y]) {
            max[Y]=acc_data[Y];
        } else if (acc_data[Y] < min[Y]) {
            min[Y]=acc_data[Y]; }
        if (acc_data[Z] > max[Z]) {
            max[Z]=acc_data[Z];
        } else if (acc_data[Z] < min[Z]) {
            min[Z]=acc_data[Z]; }
        ThisThread::sleep_for(20ms);
    }
    // calculate difference: if bigger than "bound" the calibration will be restarted
    delta[X] = max[X] - min[X];
    delta[Y] = max[Y] - min[Y];
    delta[Z] = max[Z] - min[Z];

    if(delta[X]>bound || delta[Y]>bound || delta[Z]>bound) {  //MOVEMENT DETECTED
        return true;
    } else {    //calculate average
//        offset_acc[X] =  test_acc[X]/sample;
//        offset_acc[Y] =  test_acc[Y]/sample;
//        offset_acc[Z] = (test_acc[Z]/sample) - 16384; // remove gravity vector from OFFSET
//        offset_gyr[X] =  test_gyr[X]/sample;
//        offset_gyr[Y] =  test_gyr[Y]/sample;
//        offset_gyr[Z] =  test_gyr[Z]/sample;

        acc_offset[X] =  test_acc[X]/sample;
        acc_offset[Y] =  test_acc[Y]/sample;
        acc_offset[Z] = (test_acc[Z]/sample) - 16384; // remove gravity vector from OFFSET
        gyr_offset[X] =  test_gyr[X]/sample;
        gyr_offset[Y] =  test_gyr[Y]/sample;
        gyr_offset[Z] =  test_gyr[Z]/sample;
        return false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------Magnetic sensor calibration-----------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

void mag_calibration(int* hard_mag, float* soft_mag) { //TODO Nella versione precedente questa funzione era di tipo bool, ma mi sembra che il risultato venga ignorato e che non restituisca nulla, quindi la faccio void per ora

    // HARD IRON CALIBRATION
    /* magnetic calibration is done by rotating sensor in all direction
        data will be autocalibrated in flight but without a proper calibration
        new values that exceed boundaries can lead to flight failure*/

    int     counter     = 0;     //count "without changes" cycle
    bool    changes     = true;  //check for max and min update
    bool    acceptable  = false; //checks that the calibration is a sufficient
    
    int16_t new_values[3];      //data read

    while(acceptable == false) {
        ThisThread::sleep_for(50ms);
        readMagData(new_values);
        changes = false;

        // update max min values
        if (new_values[X] > mag_max[X]) {
            mag_max[X] = new_values[X];
            changes = true;
        } else if (new_values[X]< mag_min[X]) {
            mag_min[X] = new_values[X];
            changes = true;  }
        if (new_values[Y] > mag_max[Y]) {
            mag_max[Y] = new_values[Y];
            changes = true;
        } else if (new_values[Y] < mag_min[Y]) {
            mag_min[Y] = new_values[Y];
            changes = true;  }
        if (new_values[Z] > mag_max[Z]) {
            mag_max[Z] = new_values[Z];
            changes = true;
        } else if (new_values[Z] < mag_min[Z]) {
            mag_min[Z] = new_values[Z];
            changes = true;  }

        // if not update
        if (not changes) {
            counter++;
        } else {
            counter=0;
        }
        // if counter reach acceptable value the calibration is concluded
        if (counter==200) {
            acceptable=true;
        }
    }
    //calculate offset: repostion the middle point in the center
    hard_mag[X] = (mag_max[X] + mag_min[X])/2;
    hard_mag[Y] = (mag_max[Y] + mag_min[Y])/2;
    hard_mag[Z] = (mag_max[Z] + mag_min[Z])/2;
    //normalize to +/-100
    soft_mag[X] = (mag_max[X]-hard_mag[X])/100.0000;
    soft_mag[Y] = (mag_max[Y]-hard_mag[Y])/100.0000;
    soft_mag[Z] = (mag_max[Z]-hard_mag[Z])/100.0000;

}

void default_mag() {
    hard_mag[X] = default_hard_mag[X];
    hard_mag[Y] = default_hard_mag[Y];
    hard_mag[Z] = default_hard_mag[Z];
    //normalize to +/-100
    soft_mag[X] = default_soft_mag[X];
    soft_mag[Y] = default_soft_mag[X];
    soft_mag[Z] = default_soft_mag[X];
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------READ and on flight calibration of magnetic sensor------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

void readMagData_calibr(int16_t * destination, int* hard_mag, float* soft_mag) {
    int16_t values[3];          //read data
    //serviceT.start();
    readMagData(values);
    //serviceT.stop();
    //printf("read Time %d   ", serviceT.read_us());
    //serviceT.reset();

    bool changes= false;        //max/min change flag    
    if (values[X] > mag_max[X]) {
        mag_max[X] = values[X];
        changes = true;
    } else if (values[X]< mag_min[X]) {
        mag_min[X] = values[X];
        changes = true;  }
    if (values[Y] > mag_max[Y]) {
        mag_max[Y] = values[Y];
        changes = true;
    } else if (values[Y] < mag_min[Y]) {
        mag_min[Y] = values[Y];
        changes = true;  }
    if (values[Z] > mag_max[Z]) {
        mag_max[Z] = values[Z];
        changes = true;
    } else if (values[Z] < mag_min[Z]) {
        mag_min[Z] = values[Z];
        changes = true;  }

    if (changes) {      //if changes: update offset and normalize factor
        hard_mag[X] = (mag_max[X] + mag_min[X])/2;
        hard_mag[Y] = (mag_max[Y] + mag_min[Y])/2;
        hard_mag[Z] = (mag_max[Z] + mag_min[Z])/2;

        soft_mag[X] = (mag_max[X]-hard_mag[X])/100.0000;
        soft_mag[Y] = (mag_max[Y]-hard_mag[Y])/100.0000;
        soft_mag[Z] = (mag_max[Z]-hard_mag[Z])/100.0000;
    }
    destination[X] =  values[X];        //value output
    destination[Y] =  values[Y];
    destination[Z] =  values[Z];
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------OTHERS-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

float cutOff(float value, float min, float max) {
    if      (value > max) { value = max; } 
    else if (value < min) { value = min; }
    return value;
}

int cutOffInt(int value, int min, int max) {
    if      (value > max) { value = max; } 
    else if (value < min) { value = min; }
    return value;
}
