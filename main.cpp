/* 
By: Riccardo Medvescek
date: 28/2/23
License: freeware
This project is intended as an assessment exercise for an examination. The code is complete and ready 
to be used, however, tuning of the PID coefficients and complementary filters has not been done. 
Mathematical simulation or empirical calibration work would be required.

Hardware setup 

MPU9150-----FRDM KL25Z:       ( /!\ be awere: I/O pin with I2C capabilities needed)
 VDD     ->  3.3 V 
 SDA     ->  PTE0
 SCL     ->  PTE1
 GND     ->  GND
 others  ->  N.C.
RF reciever-FRDM KL25Z:       ( /!\ be awere: I/O pin with interrupt capabilities needed)
 CH 1    ->  PTD3
 CH 2    ->  PTD2 
 CH 3    ->  PTD0 
 CH 4    ->  PTD5 
ESC PWM output:               ( /!\ be awere: I/O pin with PWM capabilities needed)
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


#define PI          3.14159265
#define FREQ        200.0  //Hz    Change this value to change the frequency of the cycle  //NOTA VA VERIFICATO SE IL TELECONMANDO FUNZIONA ANCORA BENE
#define FILTERING_ORDER 4
#define PERIOD      FILTERING_ORDER*1000000/FREQ //microseconds
#define DEG2RAD     0.0174533 
#define RAD2DEG     57.2958
#define GSCF        65.5        //Gyroscope Scaling Factor
#define INT_SCAL    2607.59     //Scale radiant to int: 0.0174533 = 1Â° = 45   ||  PI = 180Â° = 8192

//default value
#define YAW         0
#define PITCH       1
#define ROLL        2
#define THROTTLE    3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define DEBUG_PRINT(x) (printf("%d ",x))
#define DEBUG_PRINTLN(x) (printf("%d\n",x))

//calibration samples
int sample=200;

//raw data
int16_t raw_acc[3];
int16_t raw_gyr[3];
int16_t raw_mag[3];

//elaboreated data
float   acc_angle[3];       //angle estimation from acc
float   gyr_angle[3];       //angle estimation from acc
float   alpha = 0.9;       //first complementary filter parameter
float   beta  = 0.5;        //second complementary filter parameter
float   gamma = 0;        //yaw speed complementary filter parameter //NOTA: ERA 0.7
float   mag_str[3];         //intermediate variable of magnetic field
float   mag[3];             //magnetometer componets on earth system
float   pitch,roll,yaw;     //final extimeted pitch roll and yaw
int     counter = 0;        //counter

int angle[3];
int previous_yaw;
int delta_yaw;
float   ang_speed[3];       //velocitÃ  angolari

//MEDIA MOBILE
FifoReg fiforeg(8);
int FifoRegNew[8] = {0, 0, 0, 0, 0, 0, 0, 0};
short int fifoRegIndex = 0;
double average_delta_yaw;

//PID coefficients (Yaw, Pitch, Roll)
//float Kp[3] = {4.0, 1.3, 1.3};   
//float Ki[3] = {0.00, 0.00, 0.00}; 
//float Kd[3] = {0, 18, 18};
float Kp[3] = {1, 1, 1};    
float Ki[3] = {0.00, 0.00, 0.00}; 
float Kd[3] = {0, .2, .2};   

//set point, errore, errore differenziale, errore integrale (Yaw, Pitch, Roll)
int set_point[3]  = {0, 0, 0};
int new_e[3]      = {0, 0, 0};
int prev_e[3]     = {0, 0, 0};
int delta_e[3]    = {0, 0, 0};
long sum_e[3]     = {0, 0, 0};
int correction[3] = {0, 0, 0};

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
//Serial pc(USBTX,USBRX,9600); // Mi pare di aver capito che non sia più necessario definire questo, in quanto printf scrive automaticamente sul seriale a 9600
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
    while(true) {
        CycleTimer.start();

        channel1.calibrate();
        channel2.calibrate();
        channel3.calibrate();
        channel4.calibrate();

        //set point: pid objective ( * 1.82 = max 10Â°)
        set_point[0]  = (channel1.read() - 500) * 1.82;
        set_point[1]  = (channel2.read() - 500) * 1.82;
        set_point[2]  = (channel4.read() - 500) * 1.82;
        read_throttle = channel3.read();

        // printf("%d, %d, %d, %d\n",set_point[0], set_point[1], read_throttle, set_point[2]);
        DEBUG_PRINT(set_point[0]);
        DEBUG_PRINT(set_point[1]);
        DEBUG_PRINT(read_throttle);
        DEBUG_PRINTLN(set_point[2]);

        CycleEnd = CycleTimer.elapsed_time().count();
        if (CycleTime < 20000) { //Questo è fisso a 50Hz
            wait_us(int(20000 - CycleTime));
        } else 
            wait_us(20000-CycleTime);
        CycleTimer.reset();
    }
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
                printf("Motore? (0-3) ");
                scanf("%d",&motore);
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

            for(int i = 0;i<200;i++) {
                CycleTimer.start();
                CycleBegin = CycleTimer.elapsed_time().count(); //Secondo me questo è sempre zero

                ESC1.pulsewidth_us(power[0]);
                ESC2.pulsewidth_us(power[1]);
                ESC3.pulsewidth_us(power[2]);
                ESC4.pulsewidth_us(power[3]);

                //CycleEnd = duration_cast<std::chrono::microseconds>(CycleTimer.elapsed_time()).count();
                //CycleTime = CycleEnd - CycleBegin;
                CycleTime = CycleTimer.elapsed_time().count();

                if (CycleTime < 20000) { //Questo è fisso a 50Hz
                    wait_us(int(20000 - CycleTime));
                } else 
                    wait_us(20000-CycleTime);
                CycleTimer.reset();
            }
        }
    }
}

void provaSensori(){
    int totalpitch = 0;
    int totalroll = 0;
    int total_yaw = 0;
    short counter = 0;
    while(true) {
        CycleTimer.start();

        readAccelData(raw_acc);
        readGyroData(raw_gyr);
        readMagData_calibr(raw_mag); //same of readMagData(raw_mag) with in-flight autocalibration added


        // Accelerometer offset correction
        raw_acc[0] -= offset_acc_main[0];
        raw_acc[1] -= offset_acc_main[1];
        raw_acc[2] -= offset_acc_main[2];

        // Gyroscope offset correction
        raw_gyr[0] -= offset_gyr_main[0];
        raw_gyr[1] -= offset_gyr_main[1];
        raw_gyr[2] -= offset_gyr_main[2];

        // Magnetometer hard and soft iron correction
        //Per il momento ignoriamo la bussola
        
        raw_mag[0] = (raw_mag[0] - hard_mag_main[0]) / soft_mag_main[0];      // per correggere l'errore si è sviluppato il modello
        raw_mag[1] = (raw_mag[1] - hard_mag_main[1]) / soft_mag_main[1];      // X_rilevato = X_offset + alpha * x_corretto
        raw_mag[2] = (raw_mag[2] - hard_mag_main[2]) / soft_mag_main[1];      // per risolvere soft iron si normalizza a 100

        // correct axsis orientation of magnetometer (magnetometer and accelerometer don't share same axis)
        mag_str[0] =  raw_mag[1];
        mag_str[1] = -raw_mag[0];
        mag_str[2] =  raw_mag[2];

        // get angle aproximation by accelerometer vector
        acc_angle[Y]    = -atan2(  raw_acc[X],  sqrtf( raw_acc[Y] * raw_acc[Y]  +  raw_acc[Z] * raw_acc[Z] )  ) ;
        acc_angle[X]    = -atan2(  raw_acc[Y],  sqrtf( raw_acc[X] * raw_acc[X]  +  raw_acc[Z] * raw_acc[Z] )  ) ;

        // get angle aproximation by angular speed integration (degrees to radiant (0.0174533)) (GSCF (65.5))
        gyr_angle[Y]   += ( raw_gyr[Y] / FREQ / GSCF) * DEG2RAD;
        gyr_angle[X]   += (-raw_gyr[X] / FREQ / GSCF) * DEG2RAD;
        gyr_angle[Z]   += ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD;

        //printf("%d,%d,", int(acc_angle[X]*1000), int(gyr_angle[X]*1000));

        // compensate gyro angle with accelerometer angle in a complementary filter (accelerometer -> LF ; gyroscope -> HF)
        gyr_angle[Y]    = gyr_angle[Y] * alpha + acc_angle[Y] * (1-alpha);
        gyr_angle[X]    = gyr_angle[X] * alpha + acc_angle[X] * (1-alpha);

        //printf("%d\n", int(gyr_angle[X]*1000));

        // compensate yawing motion in angle estimation
        gyr_angle[Y]   += gyr_angle[X] * sin( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD );
        gyr_angle[X]   -= gyr_angle[Y] * sin( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD );

        // get pitch and roll (low pass complementary filter)
        pitch  = pitch * beta + gyr_angle[Y] * (1-beta);
        roll   = roll  * beta + gyr_angle[X] * (1-beta);

        // get yaw estimation by magnetometer 
        mag[0] = mag_str[0] * cos(pitch) + mag_str[1] * sin(roll) * sin(pitch) - mag_str[2] * cos(roll) * sin(pitch);
        mag[1] = mag_str[1] * cos(roll)  + mag_str[2] * sin(roll);

        yaw    = atan2(mag[1], mag[0]) ;
        
        // scaling float values to int -> PI = 180Â° = 8192
        // good sensitivity and range 
        angle[YAW]   = yaw   *INT_SCAL;//( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL;
        total_yaw += angle[YAW];
        //angle[PITCH] = pitch *INT_SCAL;
        //angle[ROLL]  = roll  *INT_SCAL;
        totalpitch += pitch *INT_SCAL;
        totalroll  += roll  *INT_SCAL;

        //printf("\nRollio: %d\nBeccheggio: %d\nImbardata: %d",angle[ROLL], angle[PITCH], angle[YAW]);

        
        // hard to use directly the yaw angle to correct the drift
        // alternative: get the yaw anglular speed from a filtered compass which is driftless but noisy
        // and implementing another complementary filter: LP from compass and HP from gyroscope
        // In this way we obtain the asymptotic behavior to the compass angle

        // TO BE CORRECTED
        // previous angle should be filtered like current angle //I cannot understand what this means.
        // I actually see do not see any kind of filtering on current yaw measure.
        /*
        if (angle[YAW] > 8192 && previous_yaw < -8192) {
            delta_yaw = (angle[YAW] - previous_yaw - 8192);
        } else if(angle[YAW]< -8192 && previous_yaw> 8192) {
            delta_yaw = (angle[YAW] - previous_yaw + 8192);
        } else {
            delta_yaw = (previous_yaw - angle[YAW]);
        }
        */
        delta_yaw = (previous_yaw - angle[YAW]);
        if (delta_yaw > 1024) // example: 8190 - (-8190) = 16380
            delta_yaw = 16384 - delta_yaw; // 16384 - 16380 = 4
        else if (delta_yaw < -1024) //example: -8190 - 8190 = -16380
            delta_yaw = -16384 - delta_yaw; // - 16384 - (-16380) = -4 This way, delta_yaw should always stay around zero
        
        previous_yaw = angle[YAW];

        /*
        Nota: secondo me, per calcolare adeguatamente l'imbardata media è il caso di fare così:
        Se i campioni della media sono tutti superiori a 360°, li sovrascriviamo come se fossero nei pressi dello zero
        Stesso discorso per angoli negativi molto grandi. 
        Che va anche bene forse, ma come la mettiamo per il giroscopio?
        */

        //NOTA: Siccome ho deciso di implementare un altro modo, questo lo commento
        //average_yaw = fiforeg.FifoReg_shift_and_m_av(delta_yaw);
        //NOTA: Questo calcola la media sugli ultimi 8 valori:
        FifoRegNew[fifoRegIndex] = delta_yaw;
        average_delta_yaw = 0.;
        bool yaw_wrapup = false; //For yaw angles of more than 360 or less than -360 degrees.
        for (int i = 0;i < 8; i++) {
            average_delta_yaw += FifoRegNew[i];
            total_yaw += angle[YAW];
        }
        average_delta_yaw /= 8;
        fifoRegIndex++;
        fifoRegIndex = fifoRegIndex % 8;

        //complementary filter
        //NOTA: Questo pare dare una posizione angolare, più che una velocità
        //ang_speed[YAW] = (gamma) * average_delta_yaw + (1 - gamma) * ( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL;
        //Noi vogliamo una velocità angolare:
        ang_speed[YAW] = (gamma) * average_delta_yaw + (1 - gamma) * ( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL;
        //printf("%d\n",int(ang_speed[YAW]));

        if (++counter < FILTERING_ORDER) {
            continue;
        }
        
        totalpitch /= FILTERING_ORDER;
        totalroll /= FILTERING_ORDER;
        total_yaw /= FILTERING_ORDER;

        //velocità angolare sull'imbardata, da accelerometro, bussola e filtro.
        //printf("%d, %d, %d\n",int(average_delta_yaw*1024),int(( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL*1024),int(ang_speed[YAW]*1024));
        //beccheggio, rollio, imbardata data dalla bussola.
        printf("%d, %d, %d\n",totalpitch, totalroll, total_yaw);
        
        totalpitch = 0;
        totalroll = 0;
        total_yaw = 0;

        CycleEnd = CycleTimer.elapsed_time().count();
        if (CycleTime < 20000) { //Questo è fisso a 50Hz
            wait_us(int(20000 - CycleTime));
        } else 
            wait_us(20000-CycleTime);
        CycleTimer.reset();
    }
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
    /*
    printf("Inserisci i valori di Kp:\n");
    scanf("%f",&Kp[0]);
    scanf("%f",&Kp[1]);
    scanf("%f",&Kp[2]);
    printf("Valori inseriti:\nY: %f, R: %f, P: %f\n", Kp[YAW],Kp[ROLL],Kp[PITCH]);
    */
    ThisThread::sleep_for(100ms);
    
    int mode = 0;
    printf("1 --> Prova Radiocomando\n2 --> Prova Motori\n3 --> Prova Sensori\naltro --> Avvio normale\n");
    scanf("%d",&mode);
    switch(mode) {
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
        default: printf("Avvio normale...\n");
        break;
    }

//_______________________________________________________________________________________________________________________________________
//
//|||||||||||||||||||||||||||||||||||||||||||||||||  LOOP  |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//_______________________________________________________________________________________________________________________________________
    

    int totalpitch = 0;
    int totalroll = 0;
    short counter = 0;
    int read_throttle = 0;
/* */

    while(1) {
        //Cycle time counter
        CycleTimer.start();
        //CycleBegin = CycleTimer.read_us();
        CycleBegin = CycleTimer.elapsed_time().count(); //Non  è 0?
        
        // read raw data from sensors
        readAccelData(raw_acc);
        readGyroData(raw_gyr);
        readMagData_calibr(raw_mag); //same of readMagData(raw_mag) with in-flight autocalibration added

        // Accelerometer offset correction
        raw_acc[0] -= offset_acc_main[0];
        raw_acc[1] -= offset_acc_main[1];
        raw_acc[2] -= offset_acc_main[2];

        // Gyroscope offset correction
        raw_gyr[0] -= offset_gyr_main[0];
        raw_gyr[1] -= offset_gyr_main[1];
        raw_gyr[2] -= offset_gyr_main[2];

        //printf("%d, %d, %d, %d, %d, %d", raw_acc[0], raw_acc[1], raw_acc[2], raw_gyr[0], raw_gyr[1], raw_gyr[2]);

        // Magnetometer hard and soft iron correction
        //Per il momento ignoriamo la bussola
        
        raw_mag[0] = (raw_mag[0] - hard_mag_main[0]) / soft_mag_main[0];      // per correggere l'errore si è sviluppato il modello
        raw_mag[1] = (raw_mag[1] - hard_mag_main[1]) / soft_mag_main[1];      // X_rilevato = X_offset + alpha * x_corretto
        raw_mag[2] = (raw_mag[2] - hard_mag_main[2]) / soft_mag_main[1];      // per risolvere soft iron si normalizza a 100
    
        // correct axsis orientation of magnetometer (magnetometer and accelerometer don't share same axis)
        mag_str[0] =  raw_mag[1];
        mag_str[1] = -raw_mag[0];
        mag_str[2] =  raw_mag[2];
        

        // get angle aproximation by accelerometer vector
        acc_angle[Y]    = -atan2(  raw_acc[X],  sqrtf( raw_acc[Y] * raw_acc[Y]  +  raw_acc[Z] * raw_acc[Z] )  ) ;
        acc_angle[X]    = -atan2(  raw_acc[Y],  sqrtf( raw_acc[X] * raw_acc[X]  +  raw_acc[Z] * raw_acc[Z] )  ) ;

        //printf("%d\n", int(acc_angle[X]*1000));

        // get angle aproximation by angular speed integration (degrees to radiant (0.0174533)) (GSCF (65.5))
        gyr_angle[Y]   += ( raw_gyr[Y] / FREQ / GSCF) * DEG2RAD;
        gyr_angle[X]   += (-raw_gyr[X] / FREQ / GSCF) * DEG2RAD;
        gyr_angle[Z]   += ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD;

        //printf("%d,%d,", int(acc_angle[X]*1000), int(gyr_angle[X]*1000));

        // compensate gyro angle with accelerometer angle in a complementary filter (accelerometer -> LF ; gyroscope -> HF)
        gyr_angle[Y]    = gyr_angle[Y] * alpha + acc_angle[Y] * (1-alpha);
        gyr_angle[X]    = gyr_angle[X] * alpha + acc_angle[X] * (1-alpha);

        //printf("%d\n", int(gyr_angle[X]*1000));

        // compensate yawing motion in angle estimation
        gyr_angle[Y]   += gyr_angle[X] * sin( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD );
        gyr_angle[X]   -= gyr_angle[Y] * sin( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD );

        // get pitch and roll (low pass complementary filter)
        pitch  = pitch * beta + gyr_angle[Y] * (1-beta);
        roll   = roll  * beta + gyr_angle[X] * (1-beta);
    
        // get yaw estimation by magnetometer 
        mag[0] = mag_str[0] * cos(pitch) + mag_str[1] * sin(roll) * sin(pitch) - mag_str[2] * cos(roll) * sin(pitch);
        mag[1] = mag_str[1] * cos(roll)  + mag_str[2] * sin(roll);

        yaw    = atan2(mag[1], mag[0]) ;
        
        // scaling float values to int -> PI = 180Â° = 8192
        // good sensitivity and range 
        angle[YAW]   = yaw   *INT_SCAL;//( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL;
        //angle[PITCH] = pitch *INT_SCAL;
        //angle[ROLL]  = roll  *INT_SCAL;
        totalpitch += pitch *INT_SCAL;
        totalroll  += roll  *INT_SCAL;

        //printf("\nRollio: %d\nBeccheggio: %d\nImbardata: %d",angle[ROLL], angle[PITCH], angle[YAW]);

        
        // hard to use directly the yaw angle to correct the drift
        // alternative: get the yaw anglular speed from a filtered compass which is driftless but noisy
        // and implementing another complementary filter: LP from compass and HP from gyroscope
        // In this way we obtain the asymptotic behavior to the compass angle

        // TO BE CORRECTED
        // previous angle should be filtered like current angle
        if (angle[YAW] > 8192 && previous_yaw < -8192) {
            delta_yaw = (angle[YAW] - previous_yaw - 8192);
        } else if(angle[YAW]< -8192 && previous_yaw> 8192) {
            delta_yaw = (angle[YAW] - previous_yaw + 8192);
        } else {
            delta_yaw = (previous_yaw - angle[YAW]);
        }
        
        previous_yaw = angle[YAW];

        /*
        Nota: secondo me, per calcolare adeguatamente l'imbardata media è il caso di fare così:
        Se i campioni della media sono tutti superiori a 360°, li sovrascriviamo come se fossero nei pressi dello zero
        Stesso discorso per angoli negativi molto grandi. 
        Che va anche bene forse, ma come la mettiamo per il giroscopio?
        */

        //NOTA: Siccome ho deciso di implementare un altro modo, questo lo commento
        //average_yaw = fiforeg.FifoReg_shift_and_m_av(delta_yaw);
        
        //NOTA: Questo calcola la media sugli ultimi 8 valori:
        FifoRegNew[fifoRegIndex] = delta_yaw;
        average_delta_yaw = 0.;
        for (int i = 0;i < 8; i++)
            average_delta_yaw += FifoRegNew[i];
        average_delta_yaw /= 8;
        fifoRegIndex++;
        fifoRegIndex = fifoRegIndex % 8;

        //complementary filter
        //NOTA: Questo pare dare una posizione angolare, più che una velocità
        ang_speed[YAW] = (gamma) * average_delta_yaw + (1 - gamma) * ( ( raw_gyr[Z] / FREQ / GSCF) * DEG2RAD )*INT_SCAL;
        //printf("%d\n",int(ang_speed[YAW]));

       // printf("P,R,Y: %11f %11f %11f",pitch*180/PI,roll*180/PI,yaw*180/PI);
       // printf("P,R,Y: %6d %6d %6d",angle[PITCH],angle[ROLL],angle[YAW]);

        //if(serialCom) printf("Parametri letti dal radiocomando:\nLettura canale 1:\t%d\nLettura canale 2:\t%dLettura canale 4:\t%d", set_point[0], set_point[1], set_point[2]);

        //Nota: Cominciamo a provare a impostare il setpoint su 0 e vediamo che cosa succede.
        //set_point[0] = 0;
        //set_point[1] = 0;
        //set_point[2] = 0;
        
        if (++counter < FILTERING_ORDER) {
            continue;
        }

        // use the calibration factor on RF recieved data
        channel1.calibrate();
        channel2.calibrate();
        channel3.calibrate();
        channel4.calibrate();

        //set point: pid objective ( * 1.82 = max 10Â°)
        set_point[0]  = (channel1.read() - 500) * 1.82;
        set_point[1]  = (channel2.read() - 500) * 1.82;
        set_point[2]  = (channel4.read() - 500) * 1.82;

        totalpitch /= FILTERING_ORDER;
        totalroll /= FILTERING_ORDER;
        //printf("%d, %d\n",totalpitch, totalroll);

        //new error
        new_e[YAW]   = ang_speed[YAW]   - set_point[YAW];
        //new_e[PITCH] = angle[PITCH]     - set_point[PITCH];
        //new_e[ROLL]  = angle[ROLL]      - set_point[ROLL];
        new_e[PITCH] = totalpitch     - set_point[PITCH];
        new_e[ROLL]  = totalroll      - set_point[ROLL];

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
       
        //read_throttle = 500;
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
        //printf("%d, %d, %d, %d, %d\n",set_point[0], set_point[1], set_point[2], read_throttle, power[3]);
    
        //printf("%d, %d, %d, %d,        %d,        %d, %d\n",power[0],power[1],power[2],power[3], channel3.read(), correction[ROLL],correction[PITCH]);
        //ESC OUTPUT

        //printf("%d,%d,%d,%d\n",power[0],power[1],power[2],power[3]);
        
        //printf("%d, %d\n", read_throttle, power[3]);
        
        ESC1.pulsewidth_us(power[0]);
        ESC2.pulsewidth_us(power[1]);
        ESC3.pulsewidth_us(power[2]);
        ESC4.pulsewidth_us(power[3]);
        

        // Green Led Running program (toggle every 100 cycles)
        //CycleCounter++;
        //if (CycleCounter==100) { CycleCounter=0; on_off(2); }

        // Every Cycle should be 10ms long: match 50 Hertz frequency by waiting excess time
        CycleEnd = duration_cast<std::chrono::microseconds>(CycleTimer.elapsed_time()).count();
        CycleTime = CycleEnd - CycleBegin;

        if (CycleTime < PERIOD) {
            wait_us(int(PERIOD - CycleTime));
        } else {
            printf("!!! WARNING !!!\nWORKING FREQUENCY IS TOO HIGH, SYSTEM MAY NOT WORK CORRECTLY\n"); //DEBUG oppure no, non lo so
        }
        //wait_us(20000-CycleTime);
        totalpitch = 0;
        totalroll = 0;
        counter = 0;
        CycleTimer.reset();
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