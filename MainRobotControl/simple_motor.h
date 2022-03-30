/**************************************************************************
 * Lipscomb IEEE Robotics 2022
 * Motor Control Library
 * Written by Ben Powell
 * Using Adafruit Motor Shield to control 4 DC motors.
 * M2 and M4 are aligned with the 'X' axis, and M1 and M3 with the 'Y' axis.
 *
 * Yes, I'm aware this should be a proper class and all that.
 * If I somehow find extra time after getting this robot working I'll clean it up.
 *************************************************************************/

 /* Shape of field. "Up" is y+. "Down" is y-. "Left" is x-. "Right" is x+.
 Robot starts at A.

    ------------------------------------------------------
	|         ^                                          |
	|         y+                                         |
	|    < x-   x+ >                                     |
	|         y-	                                     |
	|         v                                          |
	|                -------------------------------------
	|                |
	|                |
	|                |
	|       A        |
	|                |
	|                |
	------------------

*/

#ifndef SIMPLE_MOTOR_H
#define SIMPLE_MOTOR_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "A4988.h"
#include "BasicStepperDriver.h"

//defining arm lengths:
#define ARM_PIVOT_FULL_LENGHT  200   //this should raise us to a 90degree angle
#define ARM_PIVOT_HALF_LENGTH  100   //45degree
#define ARM_EXTEND_LENGTH      500   //this raises the arm up to tree height
#define SCREW_EXTEND_LENGTH    1000

class driveMotors {
public:
  driveMotors();
  //void initialize();
  void forward();
  void reverse();
  void left();
  void right();
  void setSpeed(int speed);
  void stopMotor(int motorNum);
  void allStop();
};

class armMotors {
public:
  armMotors();
  //void initialize();
  void clockwiseSusan(int steps);
  void counterSusan(int steps);
  void turnSusan(bool dir);
  void raiseArm(int steps);
  void lowerArm(int steps);
  void setGrabber(int steps);
  void extendScrew(int steps);
  void retractScrew(int steps);
  void openClaw();
  void closeClaw();
};
/*
// Create motor object and link pin numbers to correct motors. Run once in setup
void initializeMotors();
// Drive up (y+)
void driveUp();
// Drive down (y-)
void driveDown();
// Drive left (x-)
void driveLeft();
// Drive right (x+)
void driveRight();
// Set speed of specified motor. 1 is M1, 2 is M2, 3 is M3, 4 is M4. Speed is 0-255.
void setSpeed(int motorNum, int speed);
// Stop specific motor. 1 is M1, 2 is M2, 3 is M3, 4 is M4.
void stopMotor(int motorNum);
// Stop ALL motors
void allStop();

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
*/

#endif
