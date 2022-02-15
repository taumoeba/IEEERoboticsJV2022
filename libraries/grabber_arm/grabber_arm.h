#ifndef GRABBER_ARM_H
#define GRABBER_ARM_H

#include <Arduino.h>
#include <Adafruit_Motorshield.h>
#include <Servo.h>
#include "A4988.h"
#include "BasicStepperDriver.h"

void initializeSteppers();

void clockwiseSusan(int steps);
void counterSusan(int steps);
void extendArm(int steps);
void retractArm(int steps);
void raiseGrabber();
void lowerGrabber();
void extendScrew(int steps);
void retractScrew(int steps);
void extendSolenoid();
void retractSolenoid();