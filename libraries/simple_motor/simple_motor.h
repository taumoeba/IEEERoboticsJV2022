/**************************************************************************
 * Lipscomb IEEE Robotics 2022
 * Motor Control Library
 * Written by Ben Powell
 * Using Adafruit Motor Shield to control 4 DC motors and one servo.
 * Two motors are aligned with the 'X' axis, and two with the 'Y' axis.
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

#ifndef SIMPLE_MOTOR_HPP
#define SIMPLE_MOTOR_HPP

#include <Arduino.h>
#include <Adafruit_Motorshield.h>

// Create motor object and link pin numbers to correct motors. Run once in setup
void initializeMotors();
// Drive up (y+) the specified number of centimeters
void driveUp(int cm);
// Drive down (y-) the specified number of centimeters
void driveDown(int cm);
// Drive left (x-) the specified number of centimeters
void driveLeft(int cm);
// Drive right (x+) the specified number of centimeters
void driveRight(int cm);
// Set speed of specified motor. 0 is servo, 1 is M1, 2 is M2, 3 is M3, 4 is M4.
void setSpeed(int motorNum, speed);
// Stop specific motor. 0 is servo, 1 is M1, 2 is M2, 3 is M3, 4 is M4. 
void stopMotor(int motorNum);
// Stop ALL motors
void allStop();
