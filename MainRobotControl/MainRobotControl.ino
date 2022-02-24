#include <Adafruit_MotorShield.h>

/*******************************************************************************************
 * Main program for LU JV IEEE Robotics 2022
 * Written by Ben Powell, Stephen Fulton, 
 * April 2022
 * 
 * Microcontroller: Adafruit Grand Central M4
 * Components List:
 * - Adafruit Motor Control Shield v2.3
 * - 4x DC Motor
 * - 3x Stepper Motor
 * - 1x Servo Motor
 * - Pixy2 Smart Vision Sensor
 * - 
 * 
 ******************************************************************************************/

#include "grabber_arm.h"
#include "simple_motor.h"

void setup() {
  Serial.begin(9600);
  initializeMotors();
  initializeGrabber();
}

void loop() {
  // Testing all functions

  // Main motors
  setSpeed(1,60);
  setSpeed(2,60);
  setSpeed(3,60);
  setSpeed(4,60);
  driveUp();
  delay(3000);
  driveDown();
  delay(3000);
  driveRight();
  delay(3000);
  driveLeft();
  delay(3000);
  setSpeed(1,120);
  setSpeed(2,120);
  setSpeed(3,120);
  setSpeed(4,120);
  driveUp();
  delay(1000);
  stopMotor(1);
  delay(1000);
  stopMotor(3);
  delay(1000);
  driveRight();
  delay(1000);
  stopMotor(2);
  delay(1000);
  stopMotor(4);
  delay(1000);
  driveDown();
  delay(2000);
  allStop();

  // grabber arm
  delay(5000);
  clockwiseSusan(100);
  delay(2000);
  counterSusan(100);
  delay(2000);
  extendArm(50);
  delay(2000);
  retractArm(50);
  delay(2000);
  raiseGrabber();
  delay(2000);
  lowerGrabber();
  delay(2000);
  extendScrew(100);
  delay(2000);
  retractScrew(100);
  delay(2000);
  extendSolenoid();
  delay(2000);
  retractSolenoid();
  delay(6000);
}
