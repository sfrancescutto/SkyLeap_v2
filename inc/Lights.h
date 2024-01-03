#include <chrono>
#include <cstdint>
#include "mbed.h"

#ifndef LIGHTS_H
#define LIGHTS_H

void blink(int led, int n, std::chrono::milliseconds time_ms);
void flash(int led, std::chrono::milliseconds time_ms);
void on_off(int led);
void Led_on(int led);
void Led_off(int led);
void beep(int n);
void tone(std::chrono::milliseconds time_ms, float frequency);
void shutup();

void sound_ready();
void sound_ok();
void sound_check();
void sound_start();
void sound_error();

void signal_start();
void signal_ready();
void signal_ok();
void signal_calibration(int led);
void signal_stop_calibration(int led);
void signal_error();
void signal_calibration_mag();
void signal_stop_calibration_mag();

#endif
