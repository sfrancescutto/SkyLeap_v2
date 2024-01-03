#include "Lights.h"
#include <cstdint>
#include "mbed.h"

#define TEN_MS 10ms
#define EIGHTY_MS 80ms
#define HUNDRED_MS 100ms
#define TWOHUNDRED_MS 200ms
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  493.9
#define NOTE_C5  523.3
#define NOTE_CS5 554
#define NOTE_D5  587.3
#define NOTE_DS5 622
#define NOTE_E5  659.3
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

DigitalOut LedG(PTE20);
DigitalOut LedB(PTE21);
DigitalOut LedR(PTE22);

DigitalIn SW6(PTC9,PullUp);
PwmOut Buzz(PTE23);


void blink(int led, int n, std::chrono::milliseconds time_ms) {
    if (led==1) {
        for (int i=0; i<n; i++) {
            LedR=true;
            ThisThread::sleep_for(time_ms/2);
            LedR=false;
            ThisThread::sleep_for(time_ms/2);
        }
    }
    else if (led==2) {
        for (int i=0; i<n; i++) {
            LedG=true;
            ThisThread::sleep_for(time_ms/2);
            LedG=false;
            ThisThread::sleep_for(time_ms/2);
        }
    }
    else if (led==3)  {
        for (int i=0; i<n; i++) {
            LedB=true;
            ThisThread::sleep_for(time_ms/2);
            LedB=false;
            ThisThread::sleep_for(time_ms/2);
        }
    }
}

void flash(int led, std::chrono::milliseconds time_ms) {
    if (led==1) {
        LedR=true;
        ThisThread::sleep_for(time_ms);
        LedR=false;
    }
    else if (led==2) {
        LedG=true;
        ThisThread::sleep_for(time_ms);
        LedG=false;
    }
    else if (led==3) {
        LedB=true;
        ThisThread::sleep_for(time_ms);
        LedB=false;
    }
}

void on_off(int led) {
    if (led==1) {
        LedR=!LedR;
    }
    else if (led==2) {
        LedG=!LedG;
    }
    else if (led==3) {
        LedB=!LedB;
    }
}

void Led_on(int led) {
    if (led==1) {
        LedR=true;
    }
    else if (led==2) {
        LedG=true;
    }
    else if (led==3) {
        LedB=true;
    }
}

void Led_off(int led) {
    if (led==1)  {
        LedR=false;
    }
    else if (led==2) {
        LedG=false;
    }
    else if (led==3) {
        LedB=false;
    }
}


void beep(int n) {
    if (not SW6) {
        for(int i=0; i<n; i++) {
            Buzz = 200;
            ThisThread::sleep_for(TWOHUNDRED_MS);
            Buzz = 0;
            ThisThread::sleep_for(TWOHUNDRED_MS);
        }
    }
}

void tone(std::chrono::milliseconds time_ms, float frequency) {
    Buzz.period(1.0000000/frequency);
    Buzz.pulsewidth(1.0000000/(2*frequency));
    ThisThread::sleep_for(time_ms);
    Buzz.write(0.0000);
}

void shutup() {  //if not called the buzzer will keep producing noise
    DigitalOut Buzz(PTE23);
    Buzz=false;
}

void sound_ready() {  //1UP 
    tone(135ms, 1318.5);
    ThisThread::sleep_for(TEN_MS);
    tone(135ms, 1568);
    ThisThread::sleep_for(TEN_MS);
    tone(135ms, 2637);
    ThisThread::sleep_for(TEN_MS);
    tone(135ms, 2093);
    ThisThread::sleep_for(TEN_MS);
    tone(135ms, 2349.3);
    ThisThread::sleep_for(TEN_MS);
    tone(135ms, 2793.8);
    ThisThread::sleep_for(TEN_MS);
}

void sound_ok() { // that's right
    tone(135ms, 1979.5);
    ThisThread::sleep_for(TEN_MS);
    tone(270ms, 2637);
    ThisThread::sleep_for(TEN_MS);
}

void sound_check() {
  tone(150ms,784);
  ThisThread::sleep_for(EIGHTY_MS);
  tone(150ms,2637);
  ThisThread::sleep_for(EIGHTY_MS);
  tone(150ms,2093);
  ThisThread::sleep_for(EIGHTY_MS);
}

void sound_start() {
    tone(300ms,NOTE_E5);
    ThisThread::sleep_for(EIGHTY_MS);
    tone(150ms,NOTE_B4);
    ThisThread::sleep_for(EIGHTY_MS);
    tone(150ms,NOTE_C5);
    ThisThread::sleep_for(EIGHTY_MS);
    tone(300ms,NOTE_D5);
    ThisThread::sleep_for(EIGHTY_MS);
    tone(150ms,NOTE_C5);
    ThisThread::sleep_for(EIGHTY_MS);
    tone(150ms,NOTE_B4);
    ThisThread::sleep_for(EIGHTY_MS);
    tone(300ms,NOTE_A4);
    ThisThread::sleep_for(EIGHTY_MS);
}

void sound_error() {
    tone(125ms,174.5);
    ThisThread::sleep_for(HUNDRED_MS);
    tone(125ms,110);
    ThisThread::sleep_for(HUNDRED_MS);
    tone(125ms,92.5);
    ThisThread::sleep_for(HUNDRED_MS);
}

void signal_start() {
    Led_on(2);
    if (not SW6){sound_start();}
    ThisThread::sleep_for(500ms);
    Led_off(2);
    ThisThread::sleep_for(500ms);
}

void signal_ready() {
    Led_on(2);
    Led_on(3);
    if (not SW6){sound_ready();}
    ThisThread::sleep_for(2s);
    Led_off(2);
    Led_off(3);
}

void signal_ok() {
    ThisThread::sleep_for(300ms);
    Led_on(2);
    Led_on(3);
    if (not SW6) {
        sound_ok();
    }
    ThisThread::sleep_for(300ms);
    Led_off(2);
    Led_off(3);
}

void signal_calibration(int led) {
    blink(3,3,900ms);
    ThisThread::sleep_for(HUNDRED_MS);
    Led_on(led);
    ThisThread::sleep_for(HUNDRED_MS);
}

void signal_stop_calibration(int led) {
    Led_off(led);
    signal_ok();
}

void signal_error() {
    Led_off(1);
    ThisThread::sleep_for(300ms);

    Led_on(1);
    if (not SW6) {
        tone(200ms, 294);
    }
    Led_off(1);
    ThisThread::sleep_for(TWOHUNDRED_MS);
    Led_on(1);
    if (not SW6){
        tone(200ms, 294);
    }
    Led_off(1);
    ThisThread::sleep_for(TWOHUNDRED_MS);
    Led_on(1);
    if (not SW6){
        tone(200ms, 294);
    }
    Led_off(1);
    ThisThread::sleep_for(TWOHUNDRED_MS);
}

void signal_calibration_mag() {
    blink(3,3,900ms);
    ThisThread::sleep_for(HUNDRED_MS);
    Led_on(2);
    ThisThread::sleep_for(HUNDRED_MS);
}

void signal_stop_calibration_mag() {
    Led_off(2);
    signal_ok();
}