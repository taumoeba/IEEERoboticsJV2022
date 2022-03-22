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
void raiseArm(int steps);
void lowerArm(int steps);
void setGrabber(int degrees);
void extendScrew(int steps);
void retractScrew(int steps);
void openClaw();
void closeClaw();

#endif
