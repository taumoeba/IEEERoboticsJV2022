#ifndef GRABBER_ARM_H
#define GRABBER_ARM_H

/*
* Yes, I'm aware this should be a proper class and all that.
* If I somehow find extra time after getting this robot working I'll clean it up.
*/

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "A4988.h"
#include "BasicStepperDriver.h"

void initializeGrabber();

void clockwiseSusan(int steps);
void counterSusan(int steps);
void extendArm(int steps);
void retractArm(int steps);
void raiseGrabber(int steps);
void lowerGrabber(int steps);
void extendScrew(int steps);
void retractScrew(int steps);

#endif
