#ifndef TIMECONTROL_H
#define TIMECONTROL_H
#include <Arduino.h>
#include "MotorControl.h"

void setInitialTime();
void updateTime();
void checkTime();
void updateMinute();
void updateHour();
void printTime();

#endif
