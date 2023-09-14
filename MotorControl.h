#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include <Arduino.h>

void readEncoder();
void actuate();
void motorController(bool cw, int duty_percentage, int pause_ms);
void checkPos();
void translateToMorse();

#endif
