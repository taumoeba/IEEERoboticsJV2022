/**************************************************************************
 * Lipscomb IEEE Robotics 2022
 * Motor Control Library
 * Written by Ben Powell
 * Using Adafruit Motor Shield to control 4 DC motors and one servo.
 * Two motors are aligned with the 'X' axis, and two with the 'Y' axis.
 *************************************************************************/

//  THIS IS UNFINISHED, DO NOT USE!!!

#include "simple_motor.h"

// Create motor object and link pin numbers to correct motors. Run once in setup
void initializeMotors() {
	Adafruit_Motorshield AFMS = Adafruit_Motorshield();
	Adafruit_DCMotor *M1 = AFMS.getMotor(1)
	Adafruit_DCMotor *M2 = AFMS.getMotor(2)
	Adafruit_DCMotor *M3 = AFMS.getMotor(3)
	Adafruit_DCMotor *M4 = AFMS.getMotor(4)
	
	if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
		// if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
		Serial.println("Could not find Motor Shield. Check wiring.");
		while (1);
	}
	Serial.println("Motor Shield found.");
}
// Drive up (y+) the specified number of centimeters
void driveUp(int cm) {
	
}
// Drive down (y-) the specified number of centimeters
void driveDown(int cm) {
	
}
// Drive left (x-) the specified number of centimeters
void driveLeft(int cm) {
	
}
// Drive right (x+) the specified number of centimeters
void driveRight(int cm) {
	
}
// Set speed of specified motor. 0 is servo, 1 is M1, 2 is M2, 3 is M3, 4 is M4.
void setSpeed(int motorNum, int speed) {
	switch(motorNum) {
		case 0:
			
			break;
		case 1:
			M1->setSpeed(speed);
			break;
		case 2:
			M2->setSpeed(speed);
			break;
		case 3:
			M3->setSpeed(speed);
			break;
		case 4:
			M4->setSpeed(speed);
			break;
	}
}
// Stop specific motor. 0 is servo, 1 is M1, 2 is M2, 3 is M3, 4 is M4. 
void stopMotor(int motorNum) {
	switch(motorNum) {
		case 0:
			
			break;
		case 1:
			M1->run(RELEASE);
			break;
		case 2:
			M2->run(RELEASE);
			break;
		case 3:
			M3->run(RELEASE);
			break;
		case 4:
			M4->run(RELEASE);
			break;
	}
}
// Stop ALL motors
void allStop() {
	M1->run(RELEASE);
	M2->run(RELEASE);
	M3->run(RELEASE);
	M4->run(RELEASE);
}